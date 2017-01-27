/*
 * ProgramOptions.hpp
 *
 *  Created on: 28/apr/2013
 *      Author: marco
 */

#ifndef PROGRAMOPTIONS_HPP_
#define PROGRAMOPTIONS_HPP_

#include <boost/program_options.hpp>
#include <iostream>
#include <boost/algorithm/string/erase.hpp>
#include <iomanip>
#include <boost/algorithm/string/regex.hpp>
#include <boost/algorithm/string/join.hpp>

using namespace boost;

namespace po {
using namespace program_options;

// Funzione che lancia un'eccezione se sono state richieste due opzioni in conflitto tra loro
void conflicting_options(const po::variables_map& vm, const char* opt1, const char* opt2) {
	if (vm.count(opt1) && !vm[opt1].defaulted() && vm.count(opt2) && !vm[opt2].defaulted())
		throw std::logic_error(std::string("Conflicting options '") + opt1 + "' and '" + opt2 + "'.");
}

// Funzione che lancia un'eccezione se l'opzione voluta è stata richiesta senza quella da cui dipende
void option_dependency(const po::variables_map& vm, const char* for_what, const char* required_option) {
	if (vm.count(for_what) && !vm[for_what].defaulted())
		if (vm.count(required_option) == 0 || vm[required_option].defaulted())
			throw std::logic_error(std::string("Option '") + for_what + "' requires option '" + required_option + "'.");
}

// Funzione che lancia un'eccezione se il comando che si è specificato non è fra quelli disponibili
void validate_subcommands(const po::variables_map& vm, const char* for_what,
		const vector<string>& available_subcommands) {
	if (vm.count(for_what)) {
		bool found = false;
		string command_value = vm[for_what].as<std::string>();
		string available_subcommands_value = boost::algorithm::join(available_subcommands, "','");
		for (unsigned i = 0; i < available_subcommands.size(); i++) {
			if (command_value.compare(available_subcommands.at(i)) == 0) {
				found = true;
				break;
			}
		}
		if (found == false) {
			throw std::logic_error(
					std::string("Subcommand '") + command_value
							+ "' is not a valid subcommand. Valid subcommands are: '" + available_subcommands_value
							+ "'.");
		}
	}
}
;

// Funzione che lancia un'eccezione se le opzioni per il comando non sono state specificate
void subcommand_option_dependency(const po::variables_map& vm, const char* for_what, const char* subcommand,
		const vector<string>& required_options) {
	if (vm.count(for_what) && vm[for_what].as<std::string>().compare(subcommand) == 0) {
		string for_what_value = vm[for_what].as<std::string>();
		for (unsigned i = 0; i < required_options.size(); i++) {
			if (vm.count(required_options.at(i)) == 0 || vm[required_options.at(i)].defaulted()) {
				throw std::logic_error(
						std::string("Subcommand '") + for_what_value + "' requires option '" + required_options.at(i)
								+ "'.");
			}
		}
	}
}

/********************* PROGRAM OPTIONS PRINT FORMATTER CLASSES ******************************/

const size_t LONG_NON_PREPENDED_IF_EXIST_ELSE_PREPENDED_SHORT = 0;
const size_t LONG_PREPENDED_IF_EXIST_ELSE_PREPENDED_SHORT = 1;
const size_t SHORT_PREPENDED_IF_EXIST_ELSE_LONG = 4;
const size_t SHORT_OPTION_STRING_LENGTH = 2; // -x
const size_t ADEQUATE_WIDTH_FOR_OPTION_NAME = 20;
const bool HAS_ARGUMENT = true;
const bool DOES_NOT_HAVE_ARGUMENT = false;

class CustomOptionDescription {
public:
	CustomOptionDescription(boost::shared_ptr<boost::program_options::option_description> option) :
			required_(false), hasShort_(false), hasArgument_(false), isPositional_(false) {
		if ((option->canonical_display_name(SHORT_PREPENDED_IF_EXIST_ELSE_LONG).size() == SHORT_OPTION_STRING_LENGTH)) {
			hasShort_ = true;
			optionID_ = option->canonical_display_name(SHORT_PREPENDED_IF_EXIST_ELSE_LONG);
			optionDisplayName_ = option->canonical_display_name(SHORT_PREPENDED_IF_EXIST_ELSE_LONG);
		} else {
			hasShort_ = false;
			optionID_ = option->canonical_display_name(LONG_NON_PREPENDED_IF_EXIST_ELSE_PREPENDED_SHORT);
			optionDisplayName_ = option->canonical_display_name(LONG_PREPENDED_IF_EXIST_ELSE_PREPENDED_SHORT);
		}

		boost::shared_ptr<const boost::program_options::value_semantic> semantic = option->semantic();
		required_ = semantic->is_required();
		hasArgument_ = semantic->max_tokens() > 0 ? HAS_ARGUMENT : DOES_NOT_HAVE_ARGUMENT;

		optionDescription_ = option->description();
		optionFormatName_ = option->format_name();
	}

	void checkIfPositional(const boost::program_options::positional_options_description& positionalDesc) {
		for (size_t i = 0; i < positionalDesc.max_total_count(); ++i) {
			if (optionID_ == positionalDesc.name_for_position(i)) {
				boost::algorithm::erase_all(optionDisplayName_, "-");
				isPositional_ = true;
				break;
			}

		}
	}

	std::string getOptionUsageString() {
		std::stringstream usageString;
		if (isPositional_) {
			usageString << "\t" << std::setw(ADEQUATE_WIDTH_FOR_OPTION_NAME) << std::left << optionDisplayName_ << "\t"
					<< optionDescription_;
		} else {
			usageString << "\t" << std::setw(ADEQUATE_WIDTH_FOR_OPTION_NAME) << std::left << optionFormatName_ << "\t"
					<< optionDescription_;
		}

		return usageString.str();
	}
	;

public:
	std::string optionID_;
	std::string optionDisplayName_;
	std::string optionDescription_;
	std::string optionFormatName_;
	bool required_;
	bool hasShort_;
	bool hasArgument_;
	bool isPositional_;

};

class OptionPrinter {
public:
	void addOption(const CustomOptionDescription& optionDesc) {
		optionDesc.isPositional_ ? positionalOptions_.push_back(optionDesc) : options_.push_back(optionDesc);
	}

	/** Print the single line application usage description */
	std::string usage() {
		std::stringstream usageDesc;
		/** simple flags that have a short version */
		bool firstShortOption = true;
		usageDesc << "[";
		for (std::vector<CustomOptionDescription>::iterator it = options_.begin(); it != options_.end(); ++it) {
			if (it->hasShort_ && !it->hasArgument_ && !it->required_) {
				if (firstShortOption) {
					usageDesc << "-";
					firstShortOption = false;
				}

				usageDesc << it->optionDisplayName_[1];
			}

		}
		usageDesc << "] ";

		/** simple flags that DO NOT have a short version */
		for (std::vector<CustomOptionDescription>::iterator it = options_.begin(); it != options_.end(); ++it) {
			if (!it->hasShort_ && !it->hasArgument_ && !it->required_) {
				usageDesc << "[" << it->optionDisplayName_ << "] ";
			}
		}

		/** options with arguments */
		for (std::vector<CustomOptionDescription>::iterator it = options_.begin(); it != options_.end(); ++it) {
			if (it->hasArgument_ && !it->required_) {
				usageDesc << "[" << it->optionDisplayName_ << " ARG] ";
			}

		}

		/** required options with arguments */
		for (std::vector<CustomOptionDescription>::iterator it = options_.begin(); it != options_.end(); ++it) {
			if (it->hasArgument_ && it->required_) {
				usageDesc << it->optionDisplayName_ << " ARG ";
			}

		}

		/** positional option */
		for (std::vector<CustomOptionDescription>::iterator it = positionalOptions_.begin();
				it != positionalOptions_.end(); ++it) {
			usageDesc << it->optionDisplayName_ << " ";

		}
		return usageDesc.str();
	}

	std::string positionalOptionDetails() {
		std::stringstream output;
		for (std::vector<CustomOptionDescription>::iterator it = positionalOptions_.begin();
				it != positionalOptions_.end(); ++it) {
			output << it->getOptionUsageString() << std::endl;
		}

		return output.str();
	}

	std::string optionDetails() {
		std::stringstream output;
		for (std::vector<CustomOptionDescription>::iterator it = options_.begin(); it != options_.end(); ++it) {
			output << it->getOptionUsageString() << std::endl;

		}
		return output.str();
	}

	static void printParametersDesc(const std::string& appName, std::ostream& out,
			boost::program_options::options_description desc,
			boost::program_options::positional_options_description* positionalDesc = NULL) {
		OptionPrinter optionPrinter;

		typedef std::vector<boost::shared_ptr<boost::program_options::option_description> > Options;
		Options allOptions = desc.options();
		for (Options::iterator it = allOptions.begin(); it != allOptions.end(); ++it) {
			CustomOptionDescription currOption(*it);
			if (positionalDesc) {
				currOption.checkIfPositional(*positionalDesc);
			}

			optionPrinter.addOption(currOption);

		}

		out << "-- Option Descriptions --" << std::endl << std::endl << "Subcommands:" << std::endl
				<< optionPrinter.positionalOptionDetails() << std::endl << "Option Arguments: " << std::endl
				<< optionPrinter.optionDetails() << std::endl;
	}

	static void formatRequiredOptionError(boost::program_options::required_option& error) {
		std::string currOptionName = error.get_option_name();
		boost::algorithm::erase_regex(currOptionName, boost::regex("^-+"));
		error.set_option_name(currOptionName);
	}

private:
	std::vector<CustomOptionDescription> options_;
	std::vector<CustomOptionDescription> positionalOptions_;

};
}

#endif /* PROGRAMOPTIONS_HPP_ */
