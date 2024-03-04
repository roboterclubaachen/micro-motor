#pragma once
#include "relay_update.hpp"

class RelayAnalyzerBase
{
public:
	RelayAnalyzerBase() = default;
	virtual ~RelayAnalyzerBase() = default;

	virtual void
	setData(const std::vector<RelayUpdate>& data) = 0;
	virtual bool
	calc() const = 0;
	virtual void
	dumpToCSV() const = 0;
};