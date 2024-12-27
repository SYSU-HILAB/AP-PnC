#pragma once
#include <sqlite3.h>

#include <string>
#include <vector>

namespace sq_config_reader
{
    // Base class for SQLite database operations
    class SQLiteConfigReader
    {
       protected:
        sqlite3*      db;
        sqlite3_stmt* stmt;
        std::string   db_file;
        std::string   database_name;
        std::string   table_name;

        std::vector<double> parse_csv(const std::string& csv_str);

       public:
        SQLiteConfigReader(const std::string& db_name,
                           const std::string& tbl_name);
        virtual ~SQLiteConfigReader();

        virtual bool prepare_statement(int id = 1) = 0;
        virtual bool fetch_data()                  = 0;

        bool connect();
        bool access_and_fetch_data(int id = 1);
    };

}  // namespace sq_config_reader