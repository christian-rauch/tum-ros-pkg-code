#include <ros/ros.h>
#include <dirent.h>
#include <cmath>
#include <sys/stat.h>
#include <ANN/ANN.h>
#include <math.h>
#include <fstream>
#include <time.h>
#include "rospack/rospack.h"
#include "odu_finder.h"

////////////////////////////
DocumentInfo::DocumentInfo() :
delete_document(false) {
}

//////////////////////////////////////////////////////////////////////
DocumentInfo::DocumentInfo(vt::Document* document, std::string& name) :
	                        delete_document(false), document(document), name(name) {
}

////////////////////////////
DocumentInfo::~DocumentInfo() {
    if (delete_document)
        delete[] document;
}

///////////////////////////////////////////
void DocumentInfo::write(std::ostream& out) {
    size_t length = name.length();
    out.write((char*) &length, sizeof(size_t));
    out.write(name.c_str(), name.length());
    size_t doc_length = document->size();
    out.write((char*) &doc_length, sizeof(size_t));
    out.write((char*) &(document->at(0)), doc_length * sizeof(vt::Word));
}

/////////////////////////////////////////
void DocumentInfo::read(std::istream& in) {
    size_t length;
    in.read((char*) &length, sizeof(size_t));
    char* name = new char[length + 1];
    in.read(name, length);
    name[length] = 0;
    this->name.assign(name);
    size_t doc_length;
    in.read((char*) &doc_length, sizeof(size_t));
    document = new vt::Document(doc_length);
    in.read((char*) &document->at(0), doc_length * sizeof(vt::Word));
    this->delete_document = true;
    delete[] name;
}

/////////////////////////////////////////////////////////////////////////////////////
ODUFinder::ODUFinder() :
	                        camera_image(NULL), template_image(NULL), image(NULL), tree_builder(
	                                Feature::Zero()), visualization_mode_(FRAMES) { //SEQUENCES) {  //FRAMES) {   //SEQUENCES
    command = std::string("/load");
    database_location = std::string("database/germandeli");
    images_directory = std::string("data/germandeli");
    images_for_visualization_directory = std::string("");
    votes_count = 30;
    tree_k = 5;
    tree_levels = 5;
    min_cluster_size = 30;
    unknown_object_threshold = 0.3;
    sliding_window_size = 10;
    enable_clustering = 1;
    enable_incremental_learning = 0;
    enable_visualization = 0;
    object_id = 700000;
    frame_number = 0;
    radius_adaptation_r_min = 200.0;
    radius_adaptation_r_max = 600.9;
    radius_adaptation_A = 800.0;
    radius_adaptation_K = 0.02;
    count_templates = 0;
    //		SiftParameters params = GetSiftParameters();
    //		params.DoubleImSize = 0;
    //		SetSiftParameters(params);

    // Logger
    rospack::ROSPack rp;
    char *p[] = { "rospack", "find", "objects_of_daily_use_finder" };
    //std::string p("rospack find objects_of_daily_use_finder");
    rp.run(3, p);
    //ROS_INFO("ERROOORR %s",p.c_str());
    //rp.run(p);
    char loggerFileName[300];
    //Copy string with char * strcpy ( char * destination, const char * source );
    strcpy(loggerFileName, rp.getOutput().c_str());
    //Get string length
    loggerFileName[strlen(loggerFileName) - 1] = 0;
    char timeStr[30];
    time_t t = time(NULL);
    //Format time to string with size_t strftime ( char * ptr, size_t maxsize, const char * format, const struct tm * timeptr );
    strftime(timeStr, 30, "/stat_%Y%m%d_%H%M%S.txt", localtime(&t));
    //Concatenate strings with char * strcat ( char * destination, const char * source );
    strcat(loggerFileName, timeStr);

    //ROS_INFO("Creating statistics file at: %s\n", loggerFileName);
    ROS_INFO("Creating statistics file at: %s\n", "test.txt");
    //logger.open(loggerFileName, std::fstream::out);
    logger.open ("/home/monica/test.txt", std::ofstream::app);

    //color table - used for the visualization only
    color_table[0] = cvScalar(255, 0, 0);
    color_table[1] = cvScalar(0, 255, 0);
    color_table[2] = cvScalar(0, 0, 255);
    color_table[3] = cvScalar(255, 255, 0);
    color_table[4] = cvScalar(255, 0, 255);
    color_table[5] = cvScalar(0, 255, 255);
    color_table[6] = cvScalar(255, 255, 255);
    color_table[7] = cvScalar(125, 255, 255);
    color_table[8] = cvScalar(255, 125, 255);
    color_table[9] = cvScalar(255, 255, 125);
    color_table[10] = cvScalar(125, 125, 255);
    color_table[11] = cvScalar(255, 125, 125);
    color_table[12] = cvScalar(125, 255, 125);
}

///////////////////////////////////////////////////////////////
void ODUFinder::set_visualization(bool enable_visualization_in) {
    enable_visualization = enable_visualization_in;
    if (enable_visualization) {
        cvNamedWindow("visualization", CV_WINDOW_AUTOSIZE);
        cvStartWindowThread();
    }
}

//////////////////////
ODUFinder::~ODUFinder() {
    if (enable_visualization) {
        cvReleaseImage(&camera_image);
        cvReleaseImage(&template_image);
        cvReleaseImage(&image);
        cvDestroyWindow("visualization");
    }
    delete db;
    std::map<int, DocumentInfo*>::iterator iter;
    for (iter = documents_map.begin(); iter != documents_map.end(); ++iter)
        delete[] iter->second;
    logger.close();
}

///////////////////////////////////////////////////////////////
std::string ODUFinder::process_image(IplImage* camera_image_in) {
    //extract keypoints in the whole image
    ROS_INFO("call keypoints extractor!");
    Keypoint keypoints = extract_keypoints(camera_image_in);
    Keypoint p = keypoints;
    camera_keypoints_count = 0;
    //declare vector<T*> vt;
    vt::Document full_doc;
    //push keypoints in the vocabulary tree document
    while (p != NULL) {
        Feature f(p->descrip);
        full_doc.push_back(tree.quantize(f));
        p = p->next;
        ++camera_keypoints_count;
    }
    ROS_INFO_STREAM(camera_keypoints_count << " keypoints found!");

    // cluster keypoints
    //features2d.clear();
    //features2d.resize(camera_keypoints_count);

    //double** points = new double*[camera_keypoints_count];
    ANNpointArray points;
    points = annAllocPts(camera_keypoints_count, 2);

    //initialize a vector of objects of KeypointExt (keypoints extension class)
    std::vector<KeypointExt*> camera_keypoints(camera_keypoints_count);

    p = keypoints;
    //loop over all points and insert keypoints and their quantized words
    //into camera_keypoints
    for (int i = 0; p != NULL; ++i, p = p->next) {
        if (enable_clustering) {
            //points[i] = new double[2];
            points[i][0] = p->col;
            points[i][1] = p->row;
        }
        camera_keypoints[i] = new KeypointExt(p, full_doc[i]);
    }

    size_t cluster_count = 0;
    // if clustering enabled - group features in 2D
    // according to adaptive radius criterion
    if (enable_clustering) {
        std::vector<int> membership(camera_keypoints_count);
        cluster_count = cluster_points(points, camera_keypoints_count,
                membership, radius_adaptation_r_max, radius_adaptation_r_min,
                radius_adaptation_A, radius_adaptation_K);
        ROS_INFO_STREAM("Clusters found = " << cluster_count);

        cluster_sizes.resize(cluster_count, 0);
        cluster_sizes.assign(cluster_count, 0);
        for (size_t i = 0; i < camera_keypoints_count; ++i) {
            camera_keypoints[i]->cluster = membership[i];
            ++cluster_sizes[membership[i]];
            //delete[] points[i];
        }
        delete[] points;
    }
    matches_map.clear();
    //		std::map<uint32_t, float>::iterator it;
    //		for ( it=matches_map.begin() ; it != matches_map.end(); it++ )
    //			it->second *= 0.4;

    // Search the whole image
    // vector of Match-es
    vt::Matches matches;
    //find #votes_count matches
    db->find(full_doc, votes_count + 1, matches);
    ROS_INFO("Whole image: ");

    //calculate scores over all image
    // @TODO - Why is calculation over all image necessary?
    //update_matches_map(matches, camera_keypoints_count);

    // Calculates and accumulates scores for each cluster
    for (size_t c = 0; c < cluster_count; ++c) {
        vt::Document cluster_doc;
        vt::Matches cluster_matches;

        for (size_t i = 0; i < camera_keypoints_count; ++i)
            if (camera_keypoints[i]->cluster == c)
                cluster_doc.push_back(full_doc[i]);

        if (cluster_doc.size() < (size_t) min_cluster_size)
            continue;

        db->find(cluster_doc, votes_count + 1, cluster_matches);
        ROS_INFO_STREAM("Cluster " << c << "(size = " << cluster_doc.size()	<< "):");
        update_matches_map(cluster_matches, cluster_doc.size());
    }


    // Sort the votes
    std::vector<std::pair<uint32_t, float> > votes(matches_map.size());
    std::map<uint32_t, float>::iterator iter = matches_map.begin();
    for (int i = 0; iter != matches_map.end(); ++iter, ++i) {
        votes[i].first = iter->first;
        votes[i].second = iter->second;
    }

    //print results
    ROS_INFO("RESULTS: ");
    std::sort(votes.begin(), votes.end(), compare_pairs);

    // if object is unknown either skip detection
    // or add the template to the database
    if (votes.size() > 0 && votes[0].second < unknown_object_threshold) {
        ROS_INFO("Unknown object!");
        if (enable_incremental_learning) {
            time_t rawtime;
            struct tm * timeinfo;
            char buffer[25];
            time(&rawtime);
            timeinfo = localtime(&rawtime);
            //TODO: change this to whatever object appearance are we learning for
            strftime(buffer, 25, "Object_%Y%m%d%H%M%S", timeinfo);
            std::string object_name(buffer);
            add_image_to_database(full_doc, object_name);
            ROS_INFO("Object added into database as %s!", object_name.c_str());
        }
    } else {
        for (uint i = 0; (i < votes.size() && i < (uint) documents_map.size()); ++i)
            ROS_INFO("%s, %f", documents_map[votes[i].first]->name.c_str(),	votes[i].second);

        //int current_id = votes[0].first;

        // @TODO: make this a parameter
        // classify with the sliding window
        std::string name;
        DocumentInfo** documents_to_visualize =	new DocumentInfo*[templates_to_show];
        for (int i=0; i<templates_to_show; ++i)
            documents_to_visualize[i] = NULL;

        //		if (sliding_window_size == 1) {
        //			documents_to_visualize[0] = documents_map[current_id];
        //			name = documents_map[current_id]->name;
        //		} else {
        //			sliding_window.push_back(current_id);
        //
        //			if (sliding_window.size() > sliding_window_size)
        //				sliding_window.pop_front();
        //
        //			last_templates.clear();
        //
        //			std::list<int>::iterator iter = sliding_window.begin();
        //			for (int i = 0; iter != sliding_window.end(); ++iter, ++i) {
        //				if (last_templates.find(*iter) == last_templates.end())
        //					last_templates[*iter] = 0;
        //
        //				last_templates[*iter] += sliding_window_size;// + (sliding_windows_size - i);
        //			}
        //
        //			std::map<int, int>::iterator map_iter = last_templates.begin();
        //
        //			// Get all pairs in order to sort them later
        //			std::vector<std::pair<int, int> > pairs(last_templates.size());
        //
        //			for (; map_iter != last_templates.end(); ++map_iter)
        //			{
        //				std::pair<int, int> p;
        //				p.first = map_iter->first;
        //				p.second = map_iter->second;
        //				pairs.push_back(p);
        //			}
        //
        //			std::sort(pairs.begin(), pairs.end(), compare_pairs);
        //
        //			for (int i=0; i<templates_to_show; ++i)
        //				documents_to_visualize[i] = documents_map[pairs[i].first];
        //
        //			name = documents_map[pairs[0].first]->name;
        //
        //		}


        // Log the results
        //TODO: make this optional and probably distinguish between different types of
        //image names
        ROS_INFO("Writing to the statistics file\n");
          logger << "FRAME " << frame_number << std::endl;
          frame_number++;

        std::set<std::string> unique_names;
        int added = 0;
       // for (uint i = 0; added < templates_to_show && i < votes.size(); ++i)
       // {
            DocumentInfo* d = documents_map[votes[0].first];
            size_t position = d->name.find('_');

            // int frame_number = 0;
           // ROS_INFO("documents_map %s" , d->name.c_str());

          //  if (position == std::string::npos)
           //     position = d->name.find('.');

            std::string short_name(d->name.c_str(), position);
            std::cerr << "short_name: " << short_name << std::endl;
            if (short_name == "hmilch")

            {
                //count_templates++;
                //documents_to_visualize[added] = documents_map[votes[0].first];
            logger << short_name.c_str() << "\t" << count_templates++
                    << std::endl;
            }

			/*if (unique_names.find(short_name) == unique_names.end())
			{
				unique_names.insert(short_name);
				documents_to_visualize[added] = documents_map[votes[i].first];
				logger << short_name.c_str() << "\t" << votes[i].second
						<< std::endl;

				//if (stat_summary_map.find(short_name) == stat_summary_map.end())
				//	stat_summary_map[short_name] = 0;
				//++stat_summary_map[short_name];
				added++;
			}*/
       // }
       // ROS_INFO("the name appears %d times" , count2++);

        logger << std::endl;
        logger.flush();

        //visualize
        if (enable_visualization) {
            if (visualization_mode_ == FRAMES)
                visualize(camera_image_in, documents_to_visualize, &camera_keypoints);
            else
            {
                if (documents_to_visualize[0] != NULL)
                {
                    save_result_for_sequence(documents_to_visualize[0]->name);
                }
            }

        }
        delete[] documents_to_visualize;
        //also for logging
        size_t last_slash_id = name.find_last_of("/");
        if (last_slash_id == std::string::npos)
            last_slash_id = 0;
        else
            ++last_slash_id;
        //return the name of the object, that is the correponding image name
        return name.substr(last_slash_id, name.find_last_of('.')
                - last_slash_id).c_str();
    }
    FreeKeypoints(keypoints);
    return "";
}

//////////////////////
int ODUFinder::start() {
    //if init build and save the database
    if (command.compare("/init") == 0) {
        build_database(images_directory);
        save_database(database_location);
    }
    //load previosly built database and perform recognition
    else if (command.compare("/load") == 0)
        load_database(database_location);
    //only extract sift features and save them in a specified images_directory
    else if (command.compare("/sift_only") == 0)
        process_images(images_directory);
    else
        return 1;

    return 0;
}

/////////////////////////////////////////////////////
void ODUFinder::build_database(std::string directory) {
    std::vector<FeatureVector> images;
    trace_directory(directory.c_str(), "", images);
    ROS_INFO("Preparing features for the tree...");
    FeatureVector all_features;
    for (unsigned int i = 0; i < images.size(); ++i)
        for (unsigned int j = 0; j < images[i].size(); ++j)
            all_features.push_back(images[i][j]);

    ROS_INFO_STREAM("Building a tree with " << all_features.size()
            << " nodes...");
    tree_builder.build(all_features, tree_k, tree_levels);
    tree = tree_builder.tree();
    ROS_INFO("Creating the documents...");
    docs.resize(images.size());

    for (unsigned int i = 0; i < images.size(); ++i) {
        //printf("\tImage %d\n", i);
        for (unsigned int j = 0; j < images[i].size(); ++j) {
            //printf("\t\tFeature %d\n", j);
            docs[i].push_back(tree.quantize(images[i][j]));
        }
    }

    ROS_INFO("Creating database...");
    db = new vt::Database(tree.words());
    ROS_INFO("Populating the database with the documents...");
    for (unsigned int i = 0; i < images.size(); ++i) {
        documents_map[db->insert(docs[i])] = new DocumentInfo(&(docs[i]),
                image_names[i]);
    }

    ROS_INFO("Training database...");
    db->computeTfIdfWeights(1);
    ROS_INFO("Database created!");
}

/////////////////////////////////////////////////////
void ODUFinder::process_images(std::string directory) {
    std::vector<FeatureVector> images;
    trace_directory(directory.c_str(), "", images, true);
}

/////////////////////////////////////////////////////////////////
void ODUFinder::save_database_without_tree(std::string& directory) {
    ROS_INFO("Saving documents...");
    std::string documents_file(directory);
    documents_file.append("/images.documents");
    std::ofstream out(documents_file.c_str(), std::ios::out | std::ios::binary);
    size_t map_size = documents_map.size();
    out.write((char*) &map_size, sizeof(size_t));
    std::map<int, DocumentInfo*>::iterator iter;
    for (iter = documents_map.begin(); iter != documents_map.end(); ++iter) {
        out.write((char*) &iter->first, sizeof(int));
        iter->second->write(out);
    }

    ROS_INFO("Saving weights...");
    std::string weights_file(directory);
    weights_file.append("/images.weights");
    db->saveWeights(weights_file.c_str());
    out.close();
    ROS_INFO("Done! Press Ctrl+C and roslaunch detect.launch");
}

/////////////////////////////////////////////////////
void ODUFinder::save_database(std::string& directory) {
    ROS_INFO("Saving the tree...");
    std::string tree_file(directory);
    tree_file.append("/images.tree");
    tree.save(tree_file.c_str());
    save_database_without_tree(directory);
}

/////////////////////////////////////////////////////
void ODUFinder::load_database(std::string& directory) {
    ROS_INFO("Loading the tree...");
    std::string tree_file(directory);
    tree_file.append("/images.tree");
    tree.load(tree_file.c_str());
    ROS_INFO("Initializing the database...");
    db = new vt::Database(tree.words());//, tree.splits());
    std::string documents_file(directory);
    documents_file.append("/images.documents");
    ROS_INFO("Loading the documents... (%s)", documents_file.c_str());
    std::ifstream in(documents_file.c_str(), std::ios::in | std::ios::binary);
    size_t map_size;
    in.read((char*) &map_size, sizeof(size_t));
    for (size_t i = 0; i < map_size; ++i) {
        int id;
        DocumentInfo* document_info = new DocumentInfo();
        in.read((char*) &id, sizeof(int));
        document_info->read(in);
        vt::Document* doc = document_info->document;
        int d = db->insert(*doc);
        documents_map[d] = document_info;
    }

    ROS_INFO("Loading weights...");
    std::string weights_file(directory);
    weights_file.append("/images.weights");
    db->loadWeights(weights_file.c_str());
    in.close();
    ROS_INFO("READY!");
}

///////////////////////////////////////////////////////////////////////////
void ODUFinder::add_image_to_database(vt::Document& doc, std::string& name) {
    docs.push_back(doc);
    documents_map[db->insert(doc)] = new DocumentInfo(&doc, name);
    db->computeTfIdfWeights(1);
    //TODO: Why do we not update the images.tree???
    save_database_without_tree(database_location);
}

///////////////////////////////////////////////////////////////////////////////////////
void ODUFinder::trace_directory(const char* dir, const char* prefix,
        std::vector<FeatureVector>& images, bool onlySaveImages) {
    ROS_INFO("Tracing directory: %s", dir);
    DIR *pdir = opendir(dir);
    struct dirent *pent = NULL;
    if (pdir == NULL) {
        ROS_ERROR("ERROR! Directory %s not found", dir);
        return;
    }

    while ((pent = readdir(pdir))) {
        if (strcmp(pent->d_name, ".") != 0 && strcmp(pent->d_name, "..") != 0
                && strcmp(pent->d_name, "IGNORE") != 0) {
            std::string short_filename(prefix);
            short_filename.append(pent->d_name);
            std::string filename(dir);
            filename.append(pent->d_name);
            struct stat st_buf;
            if (lstat(filename.c_str(), &st_buf) == -1) {
                ROS_ERROR("ERROR: Invalid file name %s", filename.c_str());
                ROS_ERROR("Exiting");
                exit(2);
            }

            if (S_ISDIR(st_buf.st_mode)) {
                filename.append("/");
                short_filename.append("/");
                trace_directory(filename.c_str(), short_filename.c_str(),
                        images, onlySaveImages);
            } else {
                process_file(filename, images, onlySaveImages);
                image_names.push_back(short_filename);
            }
        }
    }
    closedir(pdir);
}

//////////////////////////////////////////////////////////////////////
void ODUFinder::visualize(IplImage *camera_image_in,
        DocumentInfo** template_document_info,
        std::vector<KeypointExt*> *camera_keypoints) {
    int templates_count = 0;

    if (visualization_mode_ == FRAMES)
        templates_count = templates_to_show;
    else if (visualization_mode_ == SEQUENCES)
    {
        templates_count = sequence_buffer.size();
    }

    if (template_image != NULL)
        cvReleaseImage(&template_image);

    if (image != NULL)
        cvReleaseImage(&image);

    std::string image_file(images_for_visualization_directory);
    IplImage** template_images = new IplImage*[templates_count];
    int total_height = 0;
    int max_width = 0;


    //calculate dimensions of the display image
    for (int i = 0; i < templates_count; ++i) {
        std::string template_image_file(image_file);

        template_images[i] = NULL;

        if (visualization_mode_ == FRAMES) {
            if (template_document_info[i] != NULL) {
                template_image_file.append(
                        template_document_info[i]->name.c_str());
                template_images[i] = cvLoadImage(template_image_file.c_str(),
                        CV_LOAD_IMAGE_GRAYSCALE);
            }
        } else if (visualization_mode_ == SEQUENCES) {
            template_image_file.append(sequence_buffer[i]);
            template_images[i] = cvLoadImage(template_image_file.c_str(),
                    CV_LOAD_IMAGE_GRAYSCALE);
        }

        if (template_images[i] != NULL) {
            total_height += template_images[i]->height;
            max_width = MAX(max_width, template_images[i]->width);
        }
    }

    int height = camera_image_in->height;

    float scale_factor = ( total_height == 0 ? 0 : ((float) height) / ((float) total_height));

    if (scale_factor > 0.0001)
        max_width = (int) (max_width * scale_factor);
    else
        max_width = 0;

    IplImage* tmp_image = cvCreateImage(cvSize(camera_image_in->width
            + max_width, height), camera_image_in->depth,
            camera_image_in->nChannels);


    cvFillImage(tmp_image, 0);
    cvSetImageROI(tmp_image, cvRect(0, 0, camera_image_in->width,
            camera_image_in->height));
    cvCopy(camera_image_in, tmp_image);

    //show template images
    int last_y = 0;
    for (int i = 0; i < templates_count; ++i) {
        if (template_images[i] == NULL)
            continue;

        IplImage* tmp_template_image = cvCreateImage(cvSize(
                template_images[i]->width * scale_factor,
                template_images[i]->height * scale_factor),
                template_images[i]->depth, template_images[i]->nChannels);
        cvResize(template_images[i], tmp_template_image);
        cvSetImageROI(tmp_image, cvRect(camera_image_in->width, last_y,
                tmp_template_image->width, tmp_template_image->height));
        last_y += tmp_template_image->height;

        //free resources
        cvCopy(tmp_template_image, tmp_image);
        cvReleaseImage(&tmp_template_image);
    }

    cvResetImageROI(tmp_image);
    image = cvCreateImage(cvSize(tmp_image->width, tmp_image->height),
            tmp_image->depth, 3);
    cvCvtColor(tmp_image, image, CV_GRAY2RGB);
    cvReleaseImage(&tmp_image);
    // display camera image keypoints
    if (camera_keypoints != NULL)
    {
        for (unsigned int i = 0; i < camera_keypoints->size(); ++i) {
            if (cluster_sizes[(*camera_keypoints)[i]->cluster]
                              >= (size_t) min_cluster_size)
                cvCircle(image, cvPoint((int) ((*camera_keypoints)[i]->keypoint->col),
                        (int) ((*camera_keypoints)[i]->keypoint->row)), 3,
                        color_table[(*camera_keypoints)[i]->cluster % COLORS]);
        }
    }
    //display template keypoints
    for (int i = 0; i < templates_count; ++i) {
        if (template_images[i] == NULL)
            continue;
        //    Keypoint template_keypoints = extract_keypoints(template_images[i], true);
        //    for (int ii=0; template_keypoints != NULL; ++ii, template_keypoints = template_keypoints->next)
        //    {
        //      cvCircle(image, cvPoint((int)(camera_image_in->width + template_keypoints->col),
        //                              (int)(template_keypoints->row)), 3,
        //               color_table[1 % COLORS]); //0, 255, 0
        //    }
        //free remaining resources
        cvReleaseImage(&template_images[i]);
    }

    delete[] template_images;

    //if the region of interest around keypoints is needed
    //useful for e.g. in-hand object modeling
    if (extract_roi_)
        extract_roi(camera_image, (*camera_keypoints));
    else
        image_roi = NULL;


    // 	// display template image keypoints
    //   std::vector<KeypointExt*> template_keypoints;
    //   if (template_document_info != NULL)
    //   {
    // //    template_keypoints.resize(template_document_info->document->size());
    //     Keypoint tmp_keypoints = extract_keypoints(template_image, true);
    //     for (int i=0; tmp_keypoints != NULL; ++i, tmp_keypoints = tmp_keypoints->next)
    //     {
    //       cvCircle(image, cvPoint((int)(camera_image_in->width + tmp_keypoints->col), (int)(tmp_keypoints->row)), 3, cvScalar(0, 255, 255));
    //       //    template_keypoints[i] = new KeypointExt(tmp_keypoints, template_document_info->document->at(i));
    //     }
    //   }

    cvShowImage("visualization", image);

    // Free resources
    if (camera_keypoints != NULL)
        for (std::vector<KeypointExt*>::iterator iter = (*camera_keypoints).begin(); iter != (*camera_keypoints).end(); ++iter)
            delete *iter;
}

/////////////////////////////////////////////////////////////////////
void ODUFinder::update_matches_map(vt::Matches& matches, size_t size) {
    for (int i = 0; (i < votes_count && i < (int) matches.size()); ++i) {
        if (matches_map.count(matches[i].id) == 0)
            matches_map[matches[i].id] = 0;
        int next_i = i;
        float diff;

        do {
            ++next_i;
            diff = matches[next_i].score - matches[i].score;
        } while (diff == 0 && next_i < votes_count);

        float database_score = 2 - matches[i].score;
        float place_score = 1;
        float size_score = 1;//size/80.0;
        float score = database_score * place_score * size_score;
        matches_map[matches[i].id] += score;
        ROS_INFO("\t%f\t%f\t%f\t%s", matches[i].score, diff, score,
                documents_map[matches[i].id]->name.c_str());
    }
}

/////////////////////////////////////////////////////////////////////////////////////
void ODUFinder::process_file(std::string& filename,
        std::vector<FeatureVector>& images, bool onlySaveImages) {
    ROS_INFO("Processing file %s...", filename.c_str());
    IplImage *image = cvLoadImage((char*) filename.c_str(),
            CV_LOAD_IMAGE_GRAYSCALE);
    Keypoint keypoints = extract_keypoints(image);
    ROS_INFO("Keypoints extracted");
    FeatureVector features;
    Keypoint p = keypoints;
    int count = 0;

    while (p != NULL) {
        Feature f(p->descrip);
        features.push_back(f);
        p = p->next;
        ++count;
    }

    if (!onlySaveImages)
        images.push_back(features);
    else {
        IplImage *colour_image = cvLoadImage((char*) filename.c_str());
        p = keypoints;
        while (p != NULL) {
            cvCircle(colour_image, cvPoint((int) (p->col), (int) (p->row)), 3,
                    cvScalar(255, 255, 0));
            p = p->next;
        }
        cvSaveImage((char*) filename.c_str(), colour_image);
        cvReleaseImage(&colour_image);
    }
    cvReleaseImage(&image);
    FreeKeypoints(keypoints);
    ROS_INFO("Done! %d features found!", count);
}

///////////////////////////////////////////////////////////////////////
Keypoint ODUFinder::extract_keypoints(IplImage *image, bool frames_only) {
    Image sift_image = CreateImage(image->height, image->width);
    for (int i = 0; i < image->height; ++i) {
        uint8_t* pSrc = (uint8_t*) image->imageData + image->widthStep * i;
        float* pDst = sift_image->pixels + i * sift_image->stride;
        for (int j = 0; j < image->width; ++j)
            pDst[j] = (float) pSrc[j] * (1.0f / 255.0f);
    }

    Keypoint keypoints;
    if (frames_only)
        keypoints = GetKeypointFrames(sift_image);
    else
        keypoints = GetKeypoints(sift_image);
    DestroyAllImages();
    return keypoints;
}

/////////////////////////////////////
void ODUFinder::write_stat_summary() {
    std::vector<std::pair<std::string, int> > pairs(stat_summary_map.size());
    std::map<std::string, int>::iterator iter = stat_summary_map.begin();
    for (int i = 0; iter != stat_summary_map.end(); ++iter, ++i) {
        pairs[i].first = iter->first;
        pairs[i].second = iter->second;
    }

    std::sort(pairs.begin(), pairs.end(), compare_pairs2);
    logger << "--- SUMMARY ---" << std::endl;
    for (unsigned int i = 0; i < pairs.size(); ++i)
        logger << pairs[i].first.c_str() << "\t" << pairs[i].second
        << std::endl;
    logger.close();
}

/////////////////////////////
void ODUFinder::extract_roi(IplImage *image,
        std::vector<KeypointExt*> camera_keypoints) {
    //create a sequence storage for projected points
    CvMemStorage* stor = cvCreateMemStorage(0);
    CvSeq* seq = cvCreateSeq(CV_SEQ_ELTYPE_POINT, sizeof(CvSeq),
            sizeof(CvPoint), stor);

    for (unsigned long i = 0; i < camera_keypoints.size(); i++) {
        cv::Point2d uv;
        CvPoint pt;
        pt.x = camera_keypoints[i]->keypoint->col;
        pt.y = camera_keypoints[i]->keypoint->row;
        cvSeqPush(seq, &pt);
    }
    //draw rectangle around the points
    CvRect rect = cvBoundingRect(seq);
    ROS_DEBUG_STREAM("rect: " << rect.x << " " << rect.y << " " << rect.width
            << " " << rect.height);

    //get subimage, aka region of interest
    cvSetImageROI(image, rect);
    //sub-image
    image_roi = cvCreateImage(cvSize(rect.width, rect.height), image->depth,
            image->nChannels);

    cvCopy(image, image_roi);
    cvResetImageROI(image); // release image ROI
    return;
}

void ODUFinder::save_result_for_sequence(std::string& best_template_filename) {
    sequence_buffer.push_back(best_template_filename);
}

void ODUFinder::visualize_sequence() {

}

void ODUFinder::clear_sequence_buffer() {
    sequence_buffer.clear();
}

