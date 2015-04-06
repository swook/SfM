#include "structures.hpp"

class Pipeline {

public:
	/**
	 * Constructors
	 */
	Pipeline(std::string folder_path);

	/**
	 * Run pipeline
	 */
	void run();

private:
	std::string folder_path;
	Images      images;

	void load_images(std::string folder_path, Images& images);

};
