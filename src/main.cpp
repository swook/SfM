#include "boost/program_options.hpp"
namespace po = boost::program_options;

#include "Pipeline.cpp"

int main(int argc, char** argv)
{
	/**
	 * Argument parsing
	 */
	po::options_description desc("Available options");
	desc.add_options()
	    ("help", "Show this message")
	    ("data-path,d", po::value<std::string>(), "Input dataset folder path")
	    ("show-clouds,s", po::bool_switch()->default_value(false), "Show result point clouds")
	    ("no-save-clouds,n", po::bool_switch()->default_value(false), "Don't save cloud outputs")
	;

	po::positional_options_description p;
	p.add("data-path", -1);

	po::variables_map vm;
	po::store(po::command_line_parser(argc, argv).options(desc).positional(p).run(), vm);
	po::notify(vm);

	if (vm.size() == 0 || vm.count("help") || !vm.count("data-path")) {
		std::cout << "Usage: " << argv[0]
			<< " [options] data-path" << std::endl
			<< desc;
		return 1;
	}

	std::string folder_path = vm["data-path"].as<std::string>();
	bool show_clouds = vm["show-clouds"].as<bool>();
	bool save_clouds = !vm["no-save-clouds"].as<bool>();


	/**
	 * Pipeline
	 */

	// Create instance of pipeline
	Pipeline pipeline(folder_path);
	// and run
	pipeline.run(save_clouds, show_clouds);


	/**
	 * End
	 */
	return 0;

}
