#pragma once

#include <sq_config_reader/sq_config_reader.hpp>

namespace sq_config_reader
{
    class BSplineAeroConfigReader : public SQLiteConfigReader
    {
       private:
        std::vector<double> cx_coefs_;
        std::vector<double> cx_knots_;
        std::vector<double> cz_coefs_;
        std::vector<double> cz_knots_;
        double              scale_factor_{1.0};  // Default to 1.0

       public:
        BSplineAeroConfigReader();
        virtual bool prepare_statement(int id = 1) override;
        virtual bool fetch_data() override;

        // Accessors
        const std::vector<double>& get_cx_coefs() const { return cx_coefs_; }
        const std::vector<double>& get_cx_knots() const { return cx_knots_; }
        const std::vector<double>& get_cz_coefs() const { return cz_coefs_; }
        const std::vector<double>& get_cz_knots() const { return cz_knots_; }
        double get_scale_factor() const { return scale_factor_; }
    };

}  // namespace sq_config_reader