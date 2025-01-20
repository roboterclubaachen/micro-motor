#include "csv_writer.hpp"

CSVWriter::CSVWriter(std::vector<std::string> columns) : cols(columns) {}

bool
CSVWriter::create(const std::filesystem::path& file)
{
	if (filestream.is_open()) return false;
	filestream = std::ofstream(file.generic_string().c_str(), std::ios::trunc | std::ios::out);
	if (!filestream.is_open()) return false;
	addRow(cols);
	return true;
}

void
CSVWriter::addRow(const std::vector<std::string>& vals)
{
	for (size_t i = 0; i < vals.size(); i++)
	{
		filestream.write(vals[i].c_str(), vals[i].size());
		if (i < vals.size() - 1) filestream.write(&delimiter, 1);
	}
	filestream.write(&linebreak, 1);
}

void
CSVWriter::close()
{
	filestream.close();
}

CSVWriter::~CSVWriter()
{
	if (filestream.is_open()) filestream.close();
}

void
CSVWriter::flush()
{
	filestream.flush();
}