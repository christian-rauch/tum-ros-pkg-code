#include <fstream>
#include <iostream>
#include <list>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <Eigen/StdVector>

#include <vocabulary_tree/simple_kmeans.h>
#include <vocabulary_tree/vocabulary_tree.h>
#include <vocabulary_tree/database.h>
#include <vocabulary_tree/tree_builder.h>
#include <vocabulary_tree/simple_kmeans.h>

#include <siftfast/siftfast.h>

#include <common.h>
using namespace odu_finder;

typedef Eigen::Matrix<float, 1, 128> Feature;
typedef std::vector<Feature, Eigen::aligned_allocator<Feature> > FeatureVector;

const int COLORS = 13;

class DocumentInfo
{
private:
	bool delete_document;
public:
	vt::Document* document;
	std::string name;
	DocumentInfo();
	DocumentInfo(vt::Document* document, std::string& name);
	~DocumentInfo();
	void write (std::ostream& out);
	void read(std::istream& in);
};

class ODUFinder
{
public:
	enum VisualizationMode { FRAMES, SEQUENCES };

	// LOGGING
	std::ofstream logger;
  bool enable_logging_;
	int frame_number;
	std::map<std::string, int> stat_summary_map;
    bool extract_roi_;

	// VISUALIZATION
	IplImage *camera_image, *template_image, *image ,*image_roi;
	CvScalar color_table[COLORS];
	std::vector<unsigned int> cluster_sizes;
	std::string output_image_topic_;
	VisualizationMode visualization_mode_;
	std::vector<std::string> sequence_buffer;

	// DATABASE
	std::vector<vt::Document> docs;
	vt::TreeBuilder<Feature> tree_builder;
	vt::VocabularyTree<Feature> tree;
	vt::Database* db;
	std::vector<std::string> image_names;
	std::map<int, DocumentInfo*> documents_map;

	// RECOGNITION
	size_t camera_keypoints_count;
    //map of DocumentIDs and their scores in the database
	std::map<uint32_t, float> matches_map;
	std::list<int> sliding_window;
	std::map<int, int> last_templates;

	// PARAMETERS
	std::string command, database_location, images_directory, images_for_visualization_directory;
	int votes_count, tree_k, tree_levels, min_cluster_size, object_id;
	double unknown_object_threshold;
	int enable_clustering, enable_incremental_learning, enable_visualization, sliding_window_size, templates_to_show;
	double radius_adaptation_r_min, radius_adaptation_r_max, radius_adaptation_K, radius_adaptation_A;
	int count_templates;
public:
	ODUFinder();

	~ODUFinder();
  
  /** \brief Create a visualization window and start the visualization threas
   * \param enable_visualization (bool)
   */
  void set_visualization(bool enable_visualization_in);
 
  /** \brief Create a visualization window and start the visualization threas
   * \param enable_visualization
   */
  std::string process_image(IplImage* camera_image);
  
	int start();

  /** \brief builds the vocabulary tree and trains the database 
   * \param directory directory with training images
   */
	void build_database(std::string directory);

  /** \brief only called with "sift_only" command argument
      for extracting of keypoints
   * \param directory directory with training images
   */
	void process_images(std::string directory);

  /** \brief saves images.weights, images.documents (faster than save_database function)
   * \param directory target directory for database
   */  
	void save_database_without_tree(std::string& directory);

  /** \brief saves images.tree, images.weights, images.documents
   * \param directory target directory for database
   */
	void save_database(std::string& directory);

  /** \brief loads the database, that is images.tree, images.weights, images.documents
   * \param directory storage location for database
   */
	void load_database(std::string& directory);
  
  /** \brief adds new templates to the database
   * \param doc full database document
   * \param name new template name (object + time stamp in this case)
   */
	void add_image_to_database(vt::Document& doc, std::string& name);
  
  /** \brief writes the statistics of the recognition process in a package
   */
	void write_stat_summary();

  /** publish when object was detected
	* publish rect with ROI region of interest
	* extract an image from another one
	*/
	void extract_roi (IplImage *image , std::vector<KeypointExt*> camera_keypoints);


protected:
  /** \brief recursively traces the directory with images
   * \param dir parent directory
   * \param prefix 
   * \param images vector of extracted keypoints
   * \param onlySaveImages whether we only extract keypoints and save images with them 
   */
	void trace_directory(const char* dir, const char* prefix, std::vector<FeatureVector>& images, bool onlySaveImages = false);

  /** \brief visualization function
   * \param camera_image_in input camera image
   * \param template_document_info which documents to visualize
   * \param camera_keypoints a pointer to the vector containing the keypoints extracted in the input image or NULL if no keypoints are provided
   */
	void visualize(IplImage *camera_image_in, DocumentInfo** template_document_info, std::vector<KeypointExt*> *camera_keypoints);

  /** \brief Adds the current result to the sequence buffer in order to be visualized later by visualize_sequence
   * \param camera_image_in input camera image
   * \param template_document_info which documents to visualize
   * \param camera_keypoints keypoints extracted in the input image
   */
	void save_result_for_sequence(std::string &best_template_filename);

  /** \brief Visualizes the stored sequence
   */
	void visualize_sequence();

  /** \brief Clears the sequence buffer
   */
	void clear_sequence_buffer();

 
 /** \brief Accumulates the scores from the clusters and updates the map of matches 
  * (matches_map) which is a map of Document IDs  and their scores in the database. 
  * \param matches - vector of Match-es (see vocabulary tree API)
  * \param size - currently unused
  */
	void update_matches_map(vt::Matches& matches, size_t size);

  /** \brief extract keypoints from training images and optionally saves them
   * \param filename input training image
   * \param images extracted keypoints
   * \param onlySaveImages whether to save images or not
   */
	void process_file(std::string& filename, std::vector<FeatureVector>& images, bool onlySaveImages = false);

  /** \brief extracts SIFT keypoints
   * \param filename input image
   * \param frames_only currently redundant @TODO: remove??
   */
	Keypoint extract_keypoints(IplImage *image, bool frames_only = false);
};
