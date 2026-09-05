#include <iostream>
#include <sq_config_reader/advanced_lift_drag_config_reader.hpp>

namespace sq_config_reader
{
    namespace
    {
        constexpr const char* DB_FILENAME = "aero_sim_params.db";
        constexpr const char* TABLE_NAME  = "advanced_lift_drag_config";

        // Column names
        constexpr const char* SIGMOID_BLEND_COL = "sigmoid_blend";
        constexpr const char* CL_ALPHA_0_COL    = "cl_alpha_0";
        constexpr const char* CL_ALPHA_COL      = "cl_alpha";
        constexpr const char* ALPHA_STALL_COL   = "alpha_stall";
        constexpr const char* EFF_COL           = "eff";
        constexpr const char* CD_0_COL          = "cd_0";
        constexpr const char* CD_FLAT_PLATE_COL = "cd_flat_plate";
        constexpr const char* CY_BETA_COL       = "cy_beta";
        constexpr const char* CL_BETA_LOSS_COL  = "cl_beta_loss";
        constexpr const char* SCALE_FACTOR_COL  = "scale_factor";
    }  // anonymous namespace

    AdvancedLiftDragConfigReader::AdvancedLiftDragConfigReader()
        : SQLiteConfigReader(DB_FILENAME, TABLE_NAME)
    {
    }

    bool AdvancedLiftDragConfigReader::prepare_statement(int id)
    {
        std::string sql =
            "SELECT " + std::string(SIGMOID_BLEND_COL) + ", " +
            std::string(CL_ALPHA_0_COL) + ", " + std::string(CL_ALPHA_COL) +
            ", " + std::string(ALPHA_STALL_COL) + ", " + std::string(EFF_COL) +
            ", " + std::string(CD_0_COL) + ", " +
            std::string(CD_FLAT_PLATE_COL) + ", " + std::string(CY_BETA_COL) +
            ", " + std::string(CL_BETA_LOSS_COL) + ", " +
            std::string(SCALE_FACTOR_COL) + " FROM " + table_name +
            " WHERE id = ?";

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

    bool AdvancedLiftDragConfigReader::fetch_data()
    {
        int rc = sqlite3_step(stmt);
        if (rc == SQLITE_ROW)
        {
            // Read each column and parse the values
            const char* sigmoid_blend_str =
                reinterpret_cast<const char*>(sqlite3_column_text(stmt, 0));
            const char* cl_alpha_0_str =
                reinterpret_cast<const char*>(sqlite3_column_text(stmt, 1));
            const char* cl_alpha_str =
                reinterpret_cast<const char*>(sqlite3_column_text(stmt, 2));
            const char* alpha_stall_str =
                reinterpret_cast<const char*>(sqlite3_column_text(stmt, 3));
            const char* eff_str =
                reinterpret_cast<const char*>(sqlite3_column_text(stmt, 4));
            const char* cd_0_str =
                reinterpret_cast<const char*>(sqlite3_column_text(stmt, 5));
            const char* cd_flat_plate_str =
                reinterpret_cast<const char*>(sqlite3_column_text(stmt, 6));
            const char* cy_beta_str =
                reinterpret_cast<const char*>(sqlite3_column_text(stmt, 7));
            const char* cl_beta_loss_str =
                reinterpret_cast<const char*>(sqlite3_column_text(stmt, 8));
            const char* scale_factor_str =
                reinterpret_cast<const char*>(sqlite3_column_text(stmt, 9));

            // Check for NULL values
            if (!sigmoid_blend_str || !cl_alpha_0_str || !cl_alpha_str ||
                !alpha_stall_str || !eff_str || !cd_0_str ||
                !cd_flat_plate_str || !cy_beta_str || !cl_beta_loss_str ||
                !scale_factor_str)
            {
                std::cerr << "Error: Missing or NULL data in the database."
                          << std::endl;
                return false;
            }

            // Parse values (taking first value from each CSV string)
            auto sigmoid_blend_vec = parse_csv(sigmoid_blend_str);
            auto cl_alpha_0_vec    = parse_csv(cl_alpha_0_str);
            auto cl_alpha_vec      = parse_csv(cl_alpha_str);
            auto alpha_stall_vec   = parse_csv(alpha_stall_str);
            auto eff_vec           = parse_csv(eff_str);
            auto cd_0_vec          = parse_csv(cd_0_str);
            auto cd_flat_plate_vec = parse_csv(cd_flat_plate_str);
            auto cy_beta_vec       = parse_csv(cy_beta_str);
            auto cl_beta_loss_vec  = parse_csv(cl_beta_loss_str);
            auto scale_factor_vec  = parse_csv(scale_factor_str);

            // Validate all vectors have at least one value
            if (sigmoid_blend_vec.empty() || cl_alpha_0_vec.empty() ||
                cl_alpha_vec.empty() || alpha_stall_vec.empty() ||
                eff_vec.empty() || cd_0_vec.empty() ||
                cd_flat_plate_vec.empty() || cy_beta_vec.empty() ||
                cl_beta_loss_vec.empty() || scale_factor_vec.empty())
            {
                std::cerr
                    << "Error: One or more values are empty after parsing."
                    << std::endl;
                return false;
            }

            // Assign the values
            sigmoid_blend_ = sigmoid_blend_vec[0];
            cl_alpha_0_    = cl_alpha_0_vec[0];
            cl_alpha_      = cl_alpha_vec[0];
            alpha_stall_   = alpha_stall_vec[0];
            eff_           = eff_vec[0];
            cd_0_          = cd_0_vec[0];
            cd_flat_plate_ = cd_flat_plate_vec[0];
            cy_beta_       = cy_beta_vec[0];
            cl_beta_loss_  = cl_beta_loss_vec[0];
            scale_factor_  = scale_factor_vec[0];

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