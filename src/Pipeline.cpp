#include "Pipeline.hpp"

Pipeline::Pipeline(std::string folder_path)
: folder_path(folder_path)
{
}

void Pipeline::run()
{
	Images images(0);

	load_images(folder_path, images);

}
