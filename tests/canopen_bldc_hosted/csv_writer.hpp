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
	writeRow(const std::vector<std::string>& vals);

	template<typename... Args>
	void
	addRow(size_t index, Args... args);

	template<typename... Args>
	void
	addRowC(size_t index, Args... args);

	void
	flush();

	void
	close();

private:
	std::vector<std::string> lastAdded{}, lastWritten{};
	std::ofstream filestream;
	const std::vector<std::string> cols;
};

template<typename... Args>
void
CSVWriter::addRow(size_t index, Args... args)
{
	std::vector<std::string> vals = {std::to_string(index), std::to_string(args)...};
	writeRow(vals);
	lastAdded = vals;
}

template<typename... Args>
void
CSVWriter::addRowC(size_t index, Args... args)
{
	std::vector<std::string> vals = {std::to_string(index), std::to_string(args)...};
	bool newEqualsLastAdded = (lastAdded.size() == vals.size());
	if (newEqualsLastAdded)
	{
		for (size_t i = 1; i < vals.size(); i++)
		{
			if (lastAdded[i] != vals[i])
			{
				newEqualsLastAdded = false;
				break;
			}
		}
	}

	if (!newEqualsLastAdded)
	{
		auto shouldWriteLastAdded =
			lastWritten.size() != lastAdded.size() || lastAdded[0] != lastWritten[0];

		if (shouldWriteLastAdded) writeRow(lastAdded);
		writeRow(vals);
	}
	lastAdded = vals;
}
#endif