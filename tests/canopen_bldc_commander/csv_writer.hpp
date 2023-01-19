#ifndef CSV_WRITER_HPP
#define CSV_WRITER_HPP
#include <filesystem>
#include <initializer_list>
#include <string>
#include <ostream>
#include <fstream>
#include <vector>

class CSVWriter
{
private:
	static constexpr char delimiter = ',';
	static constexpr char linebreak = '\n';

public:
	CSVWriter(std::vector<std::string> columns);
	~CSVWriter();

	bool
	create(const std::filesystem::path& file);

	void
	addRow(const std::vector<std::string>& vals);

	void
	flush();

	void
	close();

private:
	std::ofstream filestream;
	const std::vector<std::string> cols;
};

#endif