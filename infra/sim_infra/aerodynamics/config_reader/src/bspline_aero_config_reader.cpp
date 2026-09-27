#include <spdlog/spdlog.h>

#include <iostream>
#include <sq_config_reader/bspline_aero_config_reader.hpp>
namespace sq_config_reader
{
    namespace
    {
        constexpr int default_bspline_degree = 3;
        constexpr int default_diff_between_knots_size_and_coefs_size =
            default_bspline_degree + 1;
        constexpr const char* DB_FILENAME = "aero_sim_params.db";
        constexpr const char* TABLE_NAME  = "bspline_aero_config";
        // TODO reflections: automatically generate the column names from the
        // base class `BSplineStructure`
        constexpr const char* CX_COEFS_COL     = "cx_coefs";
        constexpr const char* CX_KNOTS_COL     = "cx_knots";
        constexpr const char* CZ_COEFS_COL     = "cz_coefs";
        constexpr const char* CZ_KNOTS_COL     = "cz_knots";
        constexpr const char* SCALE_FACTOR_COL = "scale_factor";
    }  // anonymous namespace

    BSplineAeroConfigReader::BSplineAeroConfigReader()
        : SQLiteConfigReader(DB_FILENAME, TABLE_NAME)
    {
    }

    bool BSplineAeroConfigReader::prepare_statement(int id)
    {
        std::string sql = "SELECT " + std::string(CX_COEFS_COL) + ", " +
                          std::string(CX_KNOTS_COL) + ", " +
                          std::string(CZ_COEFS_COL) + ", " +
                          std::string(CZ_KNOTS_COL) + ", " +
                          std::string(SCALE_FACTOR_COL) + " FROM " +
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

    bool BSplineAeroConfigReader::fetch_data()
    {
        int rc = sqlite3_step(stmt);
        if (rc == SQLITE_ROW)
        {
            // Read each column and parse the CSV strings
            const char* cx_coefs_str =
                reinterpret_cast<const char*>(sqlite3_column_text(stmt, 0));
            const char* cx_knots_str =
                reinterpret_cast<const char*>(sqlite3_column_text(stmt, 1));
            const char* cz_coefs_str =
                reinterpret_cast<const char*>(sqlite3_column_text(stmt, 2));
            const char* cz_knots_str =
                reinterpret_cast<const char*>(sqlite3_column_text(stmt, 3));
            const char* scale_factor_str =
                reinterpret_cast<const char*>(sqlite3_column_text(stmt, 4));

            // Check for NULL values
            if (!cx_coefs_str || !cx_knots_str || !cz_coefs_str ||
                !cz_knots_str || !scale_factor_str)
            {
                std::cerr << "Error: Missing or NULL data in the database."
                          << std::endl;
                return false;
            }

            // Parse CSV strings into vectors
            cx_coefs_ = parse_csv(cx_coefs_str);
            cx_knots_ = parse_csv(cx_knots_str);
            cz_coefs_ = parse_csv(cz_coefs_str);
            cz_knots_ = parse_csv(cz_knots_str);

            // Parse scale factor (take first value since it's a single number)
            auto scale_factor_vec = parse_csv(scale_factor_str);
            if (scale_factor_vec.empty())
            {
                std::cerr << "Error: Scale factor is empty after parsing."
                          << std::endl;
                return false;
            }
            scale_factor_ = scale_factor_vec[0];

            // Validate data
            if (cx_coefs_.empty() || cx_knots_.empty() || cz_coefs_.empty() ||
                cz_knots_.empty())
            {
                std::cerr
                    << "Error: One or more vectors are empty after parsing."
                    << std::endl;
                return false;
            }

            // Additional validation: knots size should equal coefs size
            if (cx_knots_.size() !=
                    cx_coefs_.size() +
                        default_diff_between_knots_size_and_coefs_size ||
                cz_knots_.size() !=
                    cz_coefs_.size() +
                        default_diff_between_knots_size_and_coefs_size)
            {
                std::cerr << "Error: Number of knots must be consistent with "
                          << "number of coefficients for both cx and cz. "
                          << "Got cx_knots: " << cx_knots_.size()
                          << ", cx_coefs: " << cx_coefs_.size()
                          << ", cz_knots: " << cz_knots_.size()
                          << ", cz_coefs: " << cz_coefs_.size() << std::endl;
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