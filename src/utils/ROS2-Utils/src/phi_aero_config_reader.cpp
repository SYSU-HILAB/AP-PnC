#include <iostream>
#include <sq_config_reader/phi_aero_config_reader.hpp>

namespace sq_config_reader
{
    namespace
    {
        constexpr const char* DB_FILENAME             = "aero_sim_params.db";
        constexpr const char* TABLE_NAME              = "phi_aero_config";
        constexpr size_t      phi_matrix_flatten_size = 9;

        constexpr const char* CONFIG_PHI_COL = "phi_coefs";
    }  // anonymous namespace

    PhiAeroConfigReader::PhiAeroConfigReader()
        : SQLiteConfigReader(DB_FILENAME, TABLE_NAME)
    {
    }

    bool PhiAeroConfigReader::prepare_statement(int id)
    {
        std::string sql = "SELECT " + std::string(CONFIG_PHI_COL) + " FROM " +
                          table_name + " WHERE id = ?";
        if (sqlite3_prepare_v2(db, sql.c_str(), -1, &stmt, nullptr) !=
            SQLITE_OK)
        {
            std::cerr << "Failed to prepare statement: " << sqlite3_errmsg(db)
                      << std::endl;
            return false;
        }

        if (sqlite3_bind_int(stmt, 1, id) != SQLITE_OK)
        {
            std::cerr << "Failed to bind id parameter: " << sqlite3_errmsg(db)
                      << std::endl;
            return false;
        }

        return true;
    }

    bool PhiAeroConfigReader::fetch_data()
    {
        int rc = sqlite3_step(stmt);
        if (rc == SQLITE_ROW)
        {
            const char* phi_str =
                reinterpret_cast<const char*>(sqlite3_column_text(stmt, 0));

            if (!phi_str)
            {
                std::cerr << "Error: Missing or NULL data in the database."
                          << std::endl;
                return false;
            }

            phi_ = parse_csv(phi_str);

            // Check if phi_ has exactly 9 elements
            if (phi_.size() != phi_matrix_flatten_size)
            {
                std::cerr << "Error: phi_ must have exactly "
                          << phi_matrix_flatten_size << " elements, but got "
                          << phi_.size() << std::endl;
                return false;
            }

            return true;
        }
        else if (rc == SQLITE_DONE)
        {
            std::cerr << "No data found!" << std::endl;
        }
        else
        {
            std::cerr << "Error executing query: " << sqlite3_errmsg(db)
                      << std::endl;
        }
        return false;
    }

}  // namespace sq_config_reader