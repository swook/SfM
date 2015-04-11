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

	void load_images(std::string _folder_path, Images& images);

};
