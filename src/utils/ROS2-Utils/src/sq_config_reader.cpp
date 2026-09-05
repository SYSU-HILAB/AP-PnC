#include <cstdlib>     // For std::getenv
#include <filesystem>  // For std::filesystem (C++17 and later)
#include <iostream>
#include <sq_config_reader/sq_config_reader.hpp>
#include <sstream>
#include <stdexcept>

namespace sq_config_reader
{
    std::vector<double> SQLiteConfigReader::parse_csv(
        const std::string &csv_str)
    {
        std::vector<double> result;
        std::stringstream   ss(csv_str);
        std::string         item;

        while (std::getline(ss, item, ','))
        {
            try
            {
                result.push_back(std::stod(item));
            }
            catch (const std::invalid_argument &e)
            {
                std::cerr << "Invalid argument in CSV data: " << item
                          << std::endl;
            }
            catch (const std::out_of_range &e)
            {
                std::cerr << "Out of range value in CSV data: " << item
                          << std::endl;
            }
        }

        return result;
    }

    SQLiteConfigReader::SQLiteConfigReader(const std::string &db_name,
                                           const std::string &tbl_name)
        : db(nullptr),
          stmt(nullptr),
          database_name(db_name),
          table_name(tbl_name)
    {
        const char *AERO_SIM_DATA_DIR = std::getenv("AERO_SIM_DATA_DIR");
        if (!AERO_SIM_DATA_DIR)
        {
            throw std::runtime_error(
                "AERO_SIM_DATA_DIR environment variable is not set.");
        }

        db_file = std::string(AERO_SIM_DATA_DIR) + "/" + database_name;

        if (!std::filesystem::exists(db_file))
        {
            throw std::runtime_error("Database file " + db_file +
                                     " does not exist.");
        }
    }

    SQLiteConfigReader::~SQLiteConfigReader()
    {
        if (stmt)
        {
            sqlite3_finalize(stmt);
        }
        if (db)
        {
            sqlite3_close(db);
        }
    }

    bool SQLiteConfigReader::connect()
    {
        if (sqlite3_open(db_file.c_str(), &db) != SQLITE_OK)
        {
            std::cerr << "Can't open database: " << sqlite3_errmsg(db)
                      << std::endl;
            return false;
        }
        return true;
    }

    bool SQLiteConfigReader::access_and_fetch_data(int id)
    {
        // Try to connect to database
        if (!connect())
        {
            std::cerr << "Failed to connect to database" << std::endl;
            return false;
        }

        // Prepare SQL statement
        if (!prepare_statement(id))
        {
            std::cerr << "Failed to prepare SQL statement" << std::endl;
            return false;
        }

        // Fetch data from database
        if (!fetch_data())
        {
            std::cerr << "Failed to fetch data from database" << std::endl;
            return false;
        }

        return true;
    }

}  // namespace sq_config_reader
