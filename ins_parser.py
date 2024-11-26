import csv
import pandas as pd
import argparse
import math

from datetime import datetime, timedelta

# Function to parse GPS time
def parse_gps_time(gps_week, sow):
    gps_epoch = datetime(1980, 1, 6)  # GPS epoch start
    total_seconds = gps_week * 7 * 24 * 3600 + sow
    utc_time = gps_epoch + timedelta(seconds=total_seconds)
    return int(utc_time.timestamp())

def parse_inspvaxa(data):
    parsed_data = []
    lines = data.strip().split("\n")
    for line in lines:
        if line is not None and line != "":
            msg = line.split(";")
            parts_header = msg[0].split(",")
            parts = msg[1].split(",")
            gps_week = int(parts_header[5])
            sow = float(parts_header[6])  # Extract SOW (Seconds of Week)
            utc_timestamp = parse_gps_time(gps_week, sow)  # Convert GPS time to UTC
            latitude = float(parts[2])
            longitude = float(parts[3])
            altitude = float(parts[4])
            parsed_data.append({
                "timestamp": utc_timestamp,
                "latitude": latitude,
                "longitude": longitude,
                "altitude": altitude
            })
    return parsed_data

def main(input_file, output_file):
    raw_data = open(input_file, "r").read()
    parsed_data = parse_inspvaxa(raw_data)
    
    with open(output_file, "w") as f:
        writer = csv.DictWriter(f, fieldnames=["timestamp", "latitude", "longitude", "altitude"], delimiter=" ")
        for data in parsed_data:
            writer.writerow(data)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Convert CSV GPS data to KML format")
    parser.add_argument("input_file", help="Input CSV file path")
    parser.add_argument("output_file", help="Output KML file path")
    args = parser.parse_args()

    main(args.input_file, args.output_file)