#include "Parser.hpp"

#include <fstream>

// Anonymous namespace for helper functions
namespace
{
    // Split string into vector of doubles
    std::vector<double> splitLine(const std::string line)
    {
        std::vector<double> out;

        // Starting at idx 0, parse doubles until end of line
        size_t idx = 0;
        while (idx < line.length())
        {
            size_t len;
            out.push_back(std::stod(line.substr(idx), &len));
            idx += len;
        }
        return out;
    }
}

namespace ToyProblem
{
    Parser::Parser(const std::string& filename)
    {
        // Create ifstream and verify file was able to be opened
        std::ifstream input_file(filename);
        if (!input_file.is_open())
        {
            throw std::invalid_argument(std::string("Unable to open file ") + filename);
        }

        // Parse line-by-line
        std::string line;
        while (getline(input_file, line))
        {
            m_contents.push_back(splitLine(line));
        }
    }

    CovarianceParser::CovarianceParser(const std::string& filename): Parser(filename) {}
    Covariance CovarianceParser::parse()
    {
        Covariance cov;
        cov << m_contents[0][0], m_contents[0][1],
               m_contents[1][0], m_contents[1][1];
        return cov;
    }


    StateParser::StateParser(const std::string& filename): Parser(filename) {}
    State StateParser::parse()
    {
        return State(m_contents[0][0], m_contents[1][0]);
    }


    MeasurementParser::MeasurementParser(const std::string& filename): Parser(filename) {}
    Measurements MeasurementParser::parse()
    {
        Measurements measurements;
        for (size_t i = 0; i < m_contents[0].size(); ++i)
        {
            measurements.emplace_back(m_contents[0][i], m_contents[1][i]);
        }
        return measurements;
    }
} // namespace ToyProblem
