import netCDF4  # For reading NetCDF files
import numpy as np  # For numerical operations

try:
    file_path = "path/to/your/gfwed_data.nc"  # Replace with the actual path to your .nc file
    with netCDF4.Dataset(file_path, 'r') as nc_file:  # 'r' for reading

        # 1. Inspect the NetCDF file to find variable names:
        print(nc_file.variables.keys()) #prints the available variables in the netcdf file