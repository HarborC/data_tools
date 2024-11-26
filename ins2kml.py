import csv
import simplekml
import pandas as pd
import argparse
from pyproj import Transformer
import math

def main(input_file, output_file):
    utm_transformer = Transformer.from_crs("EPSG:4326", "EPSG:32650", always_xy=True)  # UTM 50N
    
    columns = [
        "UTC", "latitude", "longitude", "altitude"
    ]
    
    df_csv = pd.read_csv(input_file, delim_whitespace=True, names=columns)
    
    if df_csv.empty:
        raise ValueError("The input file is empty or not properly formatted.")

    # 创建一个 simplekml 对象
    kml = simplekml.Kml()

    freq = 1
    if freq < 1:
        raise ValueError("Sampling frequency (freq) must be a positive integer.")

    total_distance = 0.0
    path_coords = []
    first_ts = -1
    total_time = 0.0
    for ind in df_csv.index[::freq]:
        try:
            lon = df_csv.at[ind, "longitude"]
            lat = df_csv.at[ind, "latitude"]
            alt = df_csv.at[ind, "altitude"]
            ts = df_csv.at[ind, "UTC"]
            
            if first_ts < 0:
                first_ts = ts
            
            if lon == 0 or lat == 0:
                continue
            name = str(ts)

            point = kml.newpoint(name="", coords=[(lon, lat)])
            point.style.iconstyle.scale = 0.5
            
            last_one = path_coords[-1] if len(path_coords) > 0 else None
            if last_one is not None:
                last_lon, last_lat, last_alt = last_one
                curr_lon, curr_lat, curr_alt = lon, lat, alt
                x1, y1 = utm_transformer.transform(last_lon, last_lat)
                x2, y2 = utm_transformer.transform(curr_lon, curr_lat)

                # 计算欧几里得距离
                distance = math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
                total_distance += distance

            ambstatus = 1
            if ambstatus == 1:
                fix_color = simplekml.Color.yellow
                point.style.labelstyle.color = fix_color
                point.style.iconstyle.icon.href = (
                    "http://maps.google.com/mapfiles/kml/shapes/placemark_circle.png"
                ) 
                point.style.iconstyle.color = fix_color
            
            path_coords.append((lon, lat, alt))
            total_time = ts - first_ts
        except Exception as e:
            print(f"Error processing row {ind}: {e}")  
            
    if len(path_coords) > 1:  # 至少两个点才能绘制连线
        line = kml.newlinestring(name="Path")
        line.coords = path_coords
        line.style.linestyle.width = 2  # 线宽
        line.style.linestyle.color = simplekml.Color.red

    # 保存KML文件
    kml.save(output_file)
    
    print(f"KML file saved to {output_file}")
    print(f"Total distance: {total_distance:.2f} meters")
    print(f"Total time(min): {total_time/60:.2f}")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Convert CSV GPS data to KML format")
    parser.add_argument("input_file", help="Input CSV file path")
    parser.add_argument("output_file", help="Output KML file path")
    args = parser.parse_args()

    main(args.input_file, args.output_file)