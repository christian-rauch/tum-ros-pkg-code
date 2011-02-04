#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/CvBridge.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <siftfast/siftfast.h>
#include <vocabulary_tree/vocabulary_tree.h>
#include <vocabulary_tree/database.h>
#include <vocabulary_tree/tree_builder.h>
#include <vocabulary_tree/simple_kmeans.h>
#include <Eigen3/StdVector>
#include <dirent.h>
#include <vocabulary_tree/simple_kmeans.h>
#include <fstream>
#include <iostream>
#include <cmath>
#include <sys/stat.h>
#include <ANN/ANN.h>
#include <math.h>

typedef Eigen3::Matrix<float, 1, 128> Feature;
typedef std::vector<Feature, Eigen3::aligned_allocator<Feature> > FeatureVector;

const int COLORS = 13;

double logsig(double x)
{
	return 1.0/(1.0 + exp(-x));
}

int cluster_points(ANNpointArray  points, int points_count, std::vector<int>& membership, unsigned int width = 1, float r_max = 600.0f, float r_min = 200.0f, float A = 800.0f, float K = 0.02f )
{
	double radius = (1-logsig((points_count-A)*K))*(r_max-r_min) + r_min;
	//radius = (1-logsig((points_count-800.0)/50.0))*300 + 300;

	printf("\n\n NEW RADIUS = %f\n\n", radius);


	ANNidxArray	nnIdx = new ANNidx[points_count];
//	ANNdistArray dists = new ANNdist[points_count];

	ANNkd_tree* kdTree = new ANNkd_tree(points, points_count, 2);

	membership.assign(points_count, -1);

	int last_unassigned_id = 0;
	int current_cluster = 0;

	while (last_unassigned_id < points_count)
	{
		std::vector<int> points_stack;

		points_stack.push_back(last_unassigned_id);

		while (points_stack.size() > 0)
		{
//			printf("STACK SIZE = %d\n", points_stack.size());

			int current_point_id = points_stack.back();
			points_stack.pop_back();
			membership[current_point_id] = current_cluster;

			int points_found = kdTree->annkFRSearch(points[current_point_id], radius, points_count, nnIdx);
//			printf("Neighbours found = %d\n", points_found);


			int newPointsCount = 0;

			for (int i=0; i<points_found; ++i)
				if (membership[nnIdx[i]] == -1)
					++newPointsCount;

			if (newPointsCount > 3)
			{
				for (int i=0; i<points_found; ++i)
					if (membership[nnIdx[i]] == -1)
						points_stack.push_back(nnIdx[i]);
			}
		}

		++current_cluster;

		++last_unassigned_id;
		while (last_unassigned_id < points_count && membership[last_unassigned_id] != -1)
			++last_unassigned_id;
	}

	delete [] nnIdx;
//	delete [] dists;
	delete kdTree;
	annClose();

	return current_cluster;
}


class DocumentInfo
{
private:
	bool delete_document;
public:
	vt::Document* document;
	std::string name;

	DocumentInfo():
		delete_document(false)
	{}

	DocumentInfo(vt::Document* document, std::string& name):
		document(document),
		name(name),
		delete_document(false)
	{}

	~DocumentInfo()
	{
		if (delete_document)
			delete[] document;
	}

	void write (std::ostream& out)
	{
		size_t length = name.length();
		out.write((char*)&length, sizeof(size_t));
		out.write(name.c_str(), name.length());

		size_t doc_length = document->size();
		out.write((char*) &doc_length, sizeof(size_t));
		out.write((char*) &(document->at(0)), doc_length*sizeof(vt::Word));
	}

	void read(std::istream& in)
	{
		size_t length;
		in.read((char*)&length, sizeof(size_t));
		char* name = new char[length+1];
		in.read(name, length);
		name[length] = 0;
		this->name.assign(name);

		size_t doc_length;
		in.read((char*)&doc_length, sizeof(size_t));

		document = new vt::Document(doc_length);
		in.read((char*)&document->at(0), doc_length*sizeof(vt::Word));

		this->delete_document = true;

		delete[] name;
	}
};

class KeypointExt {
public:
	Keypoint keypoint;
	vt::Word word;
	unsigned int cluster;

	KeypointExt(Keypoint keypoint, vt::Word word, unsigned int cluster = 0):
		keypoint(keypoint),
		word(word),
		cluster(cluster)
	{}
};

bool compare_keypoint_ext(KeypointExt* k1, KeypointExt* k2)
{
	return (k1->word < k2->word);
}

bool compare_pairs(std::pair<uint32_t, float> p1, std::pair<uint32_t, float> p2)
{
	return (p1.second > p2.second);
}

class ODUFinder
{
private:
	// NODE
	ros::NodeHandle node_handle;
	ros::Subscriber image_subscriber;

	// VISUALIZATION
	sensor_msgs::CvBridge bridge;
	IplImage *camera_image, *template_image, *image;
	CvScalar color_table[COLORS];
	std::vector<unsigned int> cluster_sizes;
	int should_visualize;

	// DATABASE
	std::vector<vt::Document> docs;
	vt::TreeBuilder<Feature> tree_builder;
	vt::VocabularyTree<Feature> tree;
	vt::Database* db;
	std::vector<std::string> image_names;
	std::map<int, DocumentInfo*> documents_map;

	// RECOGNITION
	size_t camera_keypoints_count;
	std::map<uint32_t, float> matches_map;

	// PARAMETERS
	std::string command, database_location, images_directory, image_topic, template_topic, visualize_string, images_for_visualization_directory;
	int votes_count, tree_k, tree_levels, min_cluster_size;
	double unknown_object_threshold;
	int enable_clustering, enable_incremental_learning;
	double radius_adaptation_r_min, radius_adaptation_r_max, radius_adaptation_K, radius_adaptation_A;

public:
	ODUFinder(ros::NodeHandle &anode)
		: node_handle(anode)
	    , tree_builder(Feature::Zero())
		, camera_image(NULL)
		, template_image(NULL)
		, image(NULL)
	{
		node_handle.param ("command", command, std::string("/load"));
		node_handle.param ("database_location", database_location, std::string("database/germandeli"));
		node_handle.param ("images_folder", images_directory, std::string("data/germandeli"));
		node_handle.param ("images_for_visualization_directory", images_for_visualization_directory, std::string(""));

		node_handle.param ("image_topic", image_topic, std::string("/narrow_stereo/left/image_rect"));
		node_handle.param ("template_topic", template_topic, std::string("/TemplateName"));
		node_handle.param ("visualization", visualize_string, std::string("/true"));
		node_handle.param ("votes_count", votes_count, 10);
		node_handle.param ("tree_k", tree_k, 5);
		node_handle.param ("tree_levels", tree_levels, 5);
		node_handle.param ("min_cluster_size", min_cluster_size, 30);
		node_handle.param ("unknown_object_threshold", unknown_object_threshold, 0.3);

		node_handle.param ("enable_clustering", enable_clustering, 1);
		node_handle.param ("enable_incremental_learning", enable_incremental_learning, 0);
		
		node_handle.param ("radius_adaptation_r_min", radius_adaptation_r_min, 200.0);
		node_handle.param ("radius_adaptation_r_max", radius_adaptation_r_max, 600.9);
		node_handle.param ("radius_adaptation_A", radius_adaptation_A, 800.0);
		node_handle.param ("radius_adaptation_K", radius_adaptation_K, 0.02);

		should_visualize = (visualize_string.compare("/true") == 0);

		image_subscriber = node_handle.subscribe(image_topic, 1, &ODUFinder::image_callback, this);

		if (should_visualize)
		{
			cvNamedWindow("visualization", CV_WINDOW_AUTOSIZE);
			cvStartWindowThread();
		}

//		SiftParameters params = GetSiftParameters();
//		params.DoubleImSize = 0;
//		SetSiftParameters(params);

		color_table[0] = cvScalar(255,		0,		0);
		color_table[1] = cvScalar(0,		255,	0);
		color_table[2] = cvScalar(0,		0,		255);
		color_table[3] = cvScalar(255,		255,	0);
		color_table[4] = cvScalar(255,		0,		255);
		color_table[5] = cvScalar(0,		255,	255);
		color_table[6] = cvScalar(255,		255,	255);

		color_table[7] = cvScalar(125,		255,	255);
		color_table[8] = cvScalar(255,		125,	255);
		color_table[9] = cvScalar(255,		255,	125);

		color_table[10] = cvScalar(125,		125,	255);
		color_table[11] = cvScalar(255,		125,	125);
		color_table[12] = cvScalar(125,		255,	125);
	}

	~ODUFinder()
	{
		if (should_visualize)
		{
			cvReleaseImage(&camera_image);
			cvReleaseImage(&template_image);
			cvReleaseImage(&image);
			cvDestroyWindow("visualization");
		}

		delete db;

		std::map<int, DocumentInfo*>::iterator iter;
		for ( iter = documents_map.begin() ; iter != documents_map.end(); ++iter )
			delete[] iter->second;
	}


	int start()
	{
		if (command.compare("/init") == 0)
		{
			build_database(images_directory);

			save_database(database_location);
		}
		else if (command.compare("/load") == 0)
			load_database(database_location);
		else
			return 1;

		ros::spin();

		return 0;
	}

	void build_database(std::string directory)
	{
		std::vector<FeatureVector> images;


		trace_directory(directory.c_str(), "", images);


		ROS_INFO("Preparing features for the tree...");
		FeatureVector all_features;
		for (int i=0; i<images.size(); ++i)
			for (int j=0; j<images[i].size(); ++j)
				all_features.push_back(images[i][j]);

		ROS_INFO("Building a tree with %d nodes...", all_features.size());
		tree_builder.build(all_features, tree_k, tree_levels);
		tree = tree_builder.tree();

		ROS_INFO("Creating the documents...");

		docs.resize(images.size());

		for (unsigned int i=0; i<images.size(); ++i)
		{
			//printf("\tImage %d\n", i);
			for (unsigned int j=0; j<images[i].size(); ++j)
			{
				//printf("\t\tFeature %d\n", j);
				docs[i].push_back(tree.quantize(images[i][j]));
			}
		}


		ROS_INFO("Creating database...");
		db = new vt::Database(tree.words());

		ROS_INFO("Populating the database with the documents...");
		for (unsigned int i=0; i<images.size(); ++i)
		{
			documents_map[db->insert(docs[i])] = new DocumentInfo(&(docs[i]), image_names[i]);
		}

		ROS_INFO("Training database...");
		db->computeTfIdfWeights(1);


		ROS_INFO("Database created!");
	}

	void save_database_without_tree(std::string& directory)
	{
		ROS_INFO("Saving documents...");
		std::string documents_file(directory);
		documents_file.append("/images.documents");

		std::ofstream out(documents_file.c_str(), std::ios::out | std::ios::binary);

		size_t map_size = documents_map.size();
		out.write((char*) &map_size, sizeof(size_t));

		std::map<int, DocumentInfo*>::iterator iter;
		for ( iter = documents_map.begin() ; iter != documents_map.end(); ++iter )
		{
			out.write((char*)&iter->first, sizeof(int));
			iter->second->write(out);
		}


		ROS_INFO("Saving weights...");
		std::string weights_file(directory);
		weights_file.append("/images.weights");
		db->saveWeights(weights_file.c_str());

		out.close();

		ROS_INFO("READY!");
	}

	void save_database(std::string& directory)
	{
		ROS_INFO("Saving the tree...");
		std::string tree_file(directory);
		tree_file.append("/images.tree");
		tree.save(tree_file.c_str());

		save_database_without_tree(directory);
	}

	void load_database(std::string& directory)
	{
		ROS_INFO("Loading the tree...");
		std::string tree_file(directory);
		tree_file.append("/images.tree");
		tree.load(tree_file.c_str());

		ROS_INFO("Initializing the database...");
		db = new vt::Database(tree.words());


		ROS_INFO("Loading the documents...");
		std::string documents_file(directory);
		documents_file.append("/images.documents");

		std::ifstream in(documents_file.c_str(), std::ios::in | std::ios::binary);

		size_t map_size;
		in.read((char*) &map_size, sizeof(size_t));

		for (int i=0; i<map_size; ++i)
		{
			int id;
			DocumentInfo* document_info = new DocumentInfo();

			in.read((char*)&id, sizeof(int));
			document_info->read(in);

			documents_map[db->insert(*(document_info->document))] = document_info;
		}

		ROS_INFO("Loading weights...");
		std::string weights_file(directory);
		weights_file.append("/images.weights");
		db->loadWeights(weights_file.c_str());

		in.close();

		ROS_INFO("READY!");
	}

	void add_image_to_database(vt::Document& doc, std::string& name)
	{
		docs.push_back(doc);
		documents_map[db->insert(doc)] = new DocumentInfo(&doc, name);

		db->computeTfIdfWeights(1);

		save_database_without_tree(database_location);
	}

private:
	void trace_directory(const char* dir, const char* prefix, std::vector<FeatureVector>& images)
	{
		ROS_INFO("Tracing directory: %s", dir);

		DIR *pdir = opendir(dir);
		struct dirent *pent = NULL;
		if (pdir == NULL)
		{
			ROS_ERROR("ERROR! Directory %s not found", dir);
			return;
		}

		while ((pent = readdir(pdir)))
		{
			if (strcmp(pent->d_name, ".") != 0  &&  strcmp(pent->d_name, "..") != 0 && strcmp(pent->d_name, "IGNORE") != 0)
			{
				std::string short_filename(prefix);
				short_filename.append(pent->d_name);
				
				std::string filename(dir);
				filename.append(pent->d_name);
				
				struct stat st_buf;
				if (lstat(filename.c_str(), &st_buf) == -1)
				{
					ROS_ERROR("ERROR!");
					return;
				}

				if (S_ISDIR(st_buf.st_mode))
				{
					filename.append("/");
					short_filename.append("/");
					trace_directory(filename.c_str(), short_filename.c_str(), images);
				}
				else
				{
					process_file(filename, images);
					image_names.push_back(short_filename);
				}
			}
		}

		closedir (pdir);
	}

	void visualize(const sensor_msgs::ImageConstPtr& received_image, DocumentInfo* template_document_info, std::vector<KeypointExt*>& camera_keypoints)
	{
		if (template_image != NULL)
			cvReleaseImage(&template_image);

		if (image != NULL)
			cvReleaseImage(&image);

		std::string image_file(images_for_visualization_directory);
		
		IplImage* template_image = NULL;
		
		if (template_document_info != NULL)
		{
			image_file.append(template_document_info->name.c_str());
			
			printf("Opening file %s...\n", image_file.c_str());
			
			template_image = cvLoadImage(image_file.c_str(), CV_LOAD_IMAGE_GRAYSCALE);
		
			if (template_image == NULL)
				return;
		}
		else
		{
			template_image = cvCreateImage(cvSize(400, 400), IPL_DEPTH_8U, 1);
			cvFillImage( template_image, 0);
		}
		

		int height = (camera_image->height > template_image->height ? camera_image->height : template_image->height);
		IplImage* tmp_image = cvCreateImage(cvSize(camera_image->width + template_image->width, height), camera_image->depth, camera_image->nChannels);
		cvFillImage(tmp_image, 0);

		cvSetImageROI(tmp_image, cvRect(0, 0, camera_image->width, camera_image->height));
		cvCopy(camera_image, tmp_image);

		cvSetImageROI(tmp_image, cvRect(camera_image->width, 0, template_image->width, template_image->height));
		cvCopy(template_image, tmp_image);

		cvResetImageROI(tmp_image);

		image = cvCreateImage(cvSize(tmp_image->width, tmp_image->height), tmp_image->depth, 3);

		cvCvtColor(tmp_image, image, CV_GRAY2RGB);

		cvReleaseImage(&tmp_image);


		// display camera image keypoints
		for (int i=0; i<camera_keypoints.size(); ++i)
		{
			if (cluster_sizes[camera_keypoints[i]->cluster] >= min_cluster_size)
				cvCircle(image, cvPoint((int)(camera_keypoints[i]->keypoint->col), (int)(camera_keypoints[i]->keypoint->row)), 3, color_table[camera_keypoints[i]->cluster % COLORS]);
		}
		//std::sort(camera_keypoints.begin(), camera_keypoints.end(), compare_keypoint_with_word);


		
		// display template image keypoints
		std::vector<KeypointExt*> template_keypoints;
		
		if (template_document_info != NULL)
		{
			template_keypoints.resize(template_document_info->document->size());
			Keypoint tmp_keypoints = extract_keypoints(template_image, true);
			for (int i=0; tmp_keypoints != NULL; ++i, tmp_keypoints = tmp_keypoints->next)
			{
				cvCircle(image, cvPoint((int)(camera_image->width + tmp_keypoints->col), (int)(tmp_keypoints->row)), 3, cvScalar(0, 255, 255));
				template_keypoints[i] = new KeypointExt(tmp_keypoints, template_document_info->document->at(i));
			}
			//std::sort(template_keypoints.begin(), template_keypoints.end(), compare_keypoint_with_word);
		}


//		// connect corresponding keypoints
//		size_t p_camera = 0, p_template = 0;
//		size_t p_max_camera = camera_keypoints.size()-1;
//		size_t p_max_template = template_keypoints.size()-1;
//		int keypoints_not_found = 0;
//		while (p_camera < p_max_camera || p_template < p_max_template)
//		{
//			int diff = camera_keypoints[p_camera]->word - template_keypoints[p_template]->word;
//
//			int range = 5;
//			if (abs(diff) < range)
//			{
//				int color = (255*abs(diff))/range;
//				cvLine(image, cvPoint(camera_keypoints[p_camera]->keypoint->col, camera_keypoints[p_camera]->keypoint->row),
//							  cvPoint(template_keypoints[p_template]->keypoint->col + camera_image->width, template_keypoints[p_template]->keypoint->row),
//							  cvScalar(color, color, 255));
//			}
//			else
//				++keypoints_not_found;
//
//			if (p_camera == p_max_camera)
//				++p_template;
//			else if (p_template == p_max_template)
//				++p_camera;
//			else
//			{
//				if (diff<0)
//					++p_camera;
//				else
//					++p_template;
//			}
//
//		}
//
//		ROS_INFO("Couldn't find correspondances for %f%% of the keypoints", 100.0 * ((float)keypoints_not_found) / (p_max_camera + p_max_template + 2.0));

		cvShowImage("visualization", image);

		// Free resources
		for (std::vector<KeypointExt*>::iterator iter = camera_keypoints.begin(); iter != camera_keypoints.end(); ++iter)
			delete *iter;
		for (std::vector<KeypointExt*>::iterator iter = template_keypoints.begin(); iter != template_keypoints.end(); ++iter)
			delete *iter;
	}

	void image_callback (const sensor_msgs::ImageConstPtr& received_image)
	{
		camera_image = bridge.imgMsgToCv(received_image, "mono8");

		printf("\n\n");
		ROS_INFO("Image received!");

		Keypoint keypoints = extract_keypoints(camera_image);
		Keypoint p = keypoints;
		camera_keypoints_count = 0;

		vt::Document full_doc;
		while (p != NULL)
		{
			Feature f(p->descrip);

			full_doc.push_back(tree.quantize(f));

			p = p->next;
			++camera_keypoints_count;
		}

		ROS_INFO("%d keypoints found!", camera_keypoints_count);


		// cluster keypoints
		//features2d.clear();
		//features2d.resize(camera_keypoints_count);

		//double** points = new double*[camera_keypoints_count];
		ANNpointArray  points;
		points = annAllocPts(camera_keypoints_count, 2);
		
		std::vector<KeypointExt*> camera_keypoints(camera_keypoints_count);

		p = keypoints;
		for (int i=0; p != NULL; ++i, p = p->next)
		{
			if (enable_clustering)
			{
			  //points[i] = new double[2];
				points[i][0] = p->col;
				points[i][1] = p->row;
			}

			camera_keypoints[i] = new KeypointExt(p, full_doc[i]);
		}


		int cluster_count = 0;

		if (enable_clustering)
		{
			std::vector<int> membership(camera_keypoints_count);


			cluster_count = cluster_points(points, camera_keypoints_count, membership, 1, radius_adaptation_r_max, radius_adaptation_r_min, radius_adaptation_A, radius_adaptation_K);

			ROS_INFO("Clusters found = %d", cluster_count);

			cluster_sizes.resize(cluster_count, 0);
			cluster_sizes.assign(cluster_count, 0);
			
			for (int i=0; i<camera_keypoints_count; ++i)
			{
				camera_keypoints[i]->cluster = membership[i];
				++cluster_sizes[membership[i]];
				delete[] points[i];
			}

			delete[] points;
		}



		matches_map.clear();

		// Search the whole image
		vt::Matches matches;
		db->find(full_doc, votes_count+1, matches);

		ROS_INFO("Whole image: ");

		update_matches_map(matches, camera_keypoints_count);


		// Search for each cluster

		for (int c=0; c<cluster_count; ++c)
		{
			vt::Document cluster_doc;
			vt::Matches cluster_matches;

			for (int i=0; i<camera_keypoints_count; ++i)
				if (camera_keypoints[i]->cluster == c)
					cluster_doc.push_back(full_doc[i]);

			if (cluster_doc.size() < min_cluster_size)
				continue;

			db->find(cluster_doc, votes_count+1, cluster_matches);

			ROS_INFO("Cluster %d (size = %d):", c, cluster_doc.size());

			update_matches_map(cluster_matches, cluster_doc.size());
		}


		// Sort the votes
		std::vector<std::pair<uint32_t, float> > votes(matches_map.size());

		std::map<uint32_t, float>::iterator iter = matches_map.begin();
		for (int i=0; iter != matches_map.end(); ++iter, ++i)
		{
			votes[i].first = iter->first;
			votes[i].second = iter->second;
		}



		ROS_INFO("RESULTS: ");

		std::sort(votes.begin(), votes.end(), compare_pairs);

		if (enable_incremental_learning && votes[0].second < unknown_object_threshold)
		{
			ROS_INFO("Unknown object!");

			time_t rawtime;
			struct tm * timeinfo;
			char buffer[25];

			time ( &rawtime );
			timeinfo = localtime ( &rawtime );

			strftime (buffer, 25, "Object_%Y%m%d%H%M%S", timeinfo);

			std::string object_name(buffer);

			add_image_to_database(full_doc, object_name);

			ROS_INFO("Object added into database as %s!", object_name.c_str());
		}
		else
		{
			for (int i=0; i<votes_count; ++i)
				ROS_INFO("%s, %f", documents_map[votes[i].first]->name.c_str(), votes[i].second);

			if (should_visualize)
			{
				//if (votes[0].second > 0.15)
					visualize(received_image, documents_map[votes[0].first], camera_keypoints);
				//else
					//visualize(received_image, NULL, camera_keypoints);
			}
		}

		FreeKeypoints(keypoints);
	}

	void update_matches_map(vt::Matches& matches, size_t size)
	{
		for (int i=0; i<votes_count; ++i)
		{
			if (matches_map.count(matches[i].id) == 0)
				matches_map[matches[i].id] = 0;

			int next_i = i;
			float diff;

			do
			{
				++next_i;
				diff = matches[next_i].score - matches[i].score;
			}
			while (diff == 0 && next_i<votes_count);


			float database_score = 2-matches[i].score;
			float place_score = 1;(votes_count - i +1);
			float size_score = 1;//size/80.0;


			float score = database_score*place_score*size_score;

			matches_map[matches[i].id] += score;

			ROS_INFO("\t%f\t%f\t%f\t%s", matches[i].score, diff, score, documents_map[matches[i].id]->name.c_str());
		}
	}

	void process_file(std::string& filename, std::vector<FeatureVector>& images)
	{
		ROS_INFO("Processing file %s...", filename.c_str());

		IplImage *image = cvLoadImage((char*) filename.c_str(), CV_LOAD_IMAGE_GRAYSCALE);

		Keypoint keypoints = extract_keypoints(image);

		ROS_INFO("Keypoints extracted");

		FeatureVector features;

		Keypoint p = keypoints;
		int count = 0;

		while (p != NULL)
		{
			Feature f(p->descrip);
			features.push_back(f);
			p = p->next;
			++count;
		}

		images.push_back(features);

		cvReleaseImage(&image);
		FreeKeypoints(keypoints);

		ROS_INFO("Done! %d features found!", count);
	}


	Keypoint extract_keypoints(IplImage *image, bool frames_only = false)
	{
		Image sift_image = CreateImage(image->height, image->width);

		for(int i = 0; i < image->height; ++i) {
			uint8_t* pSrc = (uint8_t*)image->imageData + image->widthStep*i;
			float* pDst = sift_image->pixels + i*sift_image->stride;

			for(int j = 0; j < image->width; ++j)
				pDst[j] = (float)pSrc[j]*(1.0f/255.0f);
		}


		Keypoint keypoints;

		if (frames_only)
			keypoints = GetKeypointFrames(sift_image);
		else
			keypoints = GetKeypoints(sift_image);

		DestroyAllImages();

		return keypoints;
	}
};



int main(int argc, char** argv) {
	ros::init(argc, argv, "odu_finder");

	ros::NodeHandle node_handle("~");
	ODUFinder odu_finder(node_handle);


	return odu_finder.start();
}
