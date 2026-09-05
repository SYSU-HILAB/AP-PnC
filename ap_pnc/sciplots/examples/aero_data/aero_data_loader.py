import pandas as pd

import os

cur_dir = os.path.dirname(os.path.abspath(__file__))


class AeroDataLoader:
    def __init__(self, filename): 
        self.filename = filename

# the columns are alpha_deg,cx,cz
    def read_aero_data(self):
        """Read aerodynamic data from CSV file."""
        return pd.read_csv(self.filename)
    
    def get_data(self):
        '''
        return the data in the format of alpha_deg, cx, cz
        '''
        data = self.read_aero_data()
        return data['alpha_deg'].to_numpy(), data['cx'].to_numpy(), data['cz'].to_numpy()
      
      
AeroDataLoaderInstance = AeroDataLoader(os.path.join(cur_dir, "zZhangLyuAero.csv"))


if __name__ == "__main__":
    alpha_deg, cx, cz = AeroDataLoaderInstance.get_data()
    print(alpha_deg)
    print(cx)
    print(cz)
