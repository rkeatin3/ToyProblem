#ifndef PARSER
#define PARSER

#include <string>
#include <vector>
#include <iostream>

#include "Types.hpp"

namespace ToyProblem
{

/**
 * Base class for parsing which loads file line-by-line, separating each into a vector of doubles.
 * Note that all derived classes assume files are of proper format for this toy problem.
 */
class Parser
{
 protected:
    Parser(const std::string& filename);
    virtual ~Parser() = default;

    std::vector<std::vector<double>> m_contents;
};

/**
 * Parser for retrieving covariance from a file.
 */
class CovarianceParser: public Parser
{
 public:
    CovarianceParser(const std::string& filename);
    ~CovarianceParser() = default;

    // Retrieve parsed covariance
    Covariance parse();
};

/**
 * Parser for retrieving state from a file.
 */
class StateParser: public Parser
{
 public:
    StateParser(const std::string& filename);
    ~StateParser() = default;

    // Retrieve parsed state
    State parse();
};

/**
 * Parser for retrieving set of measurements from a file.
 */
class MeasurementParser: public Parser
{
 public:
    MeasurementParser(const std::string& filename);
    ~MeasurementParser() = default;

    // Retrieve parsed measurements
    Measurements parse();
};

/**
 * Parser for retrieving set of outputs (states) from a file.
 */
using OutputParser = MeasurementParser;

} // namespace ToyProblem

#endif