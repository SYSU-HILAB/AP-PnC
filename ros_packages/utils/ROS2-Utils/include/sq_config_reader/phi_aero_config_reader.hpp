#pragma once

#include <sq_config_reader/sq_config_reader.hpp>

namespace sq_config_reader
{
    class PhiAeroConfigReader : public SQLiteConfigReader
    {
       private:
        std::vector<double> phi_;

       public:
        PhiAeroConfigReader();
        virtual bool prepare_statement(int id = 1) override;
        virtual bool fetch_data() override;

        // Accessor
        const std::vector<double>& get_phi() const { return phi_; }
    };

}  // namespace sq_config_reader