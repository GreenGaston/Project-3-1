from scapy.all import wrpcap, Ether, Raw
import struct
import csv
def points_to_pcap(points, filename):
    #function for creating test pcap file
    packets = []
    for point in points:
        payload = struct.pack('fff', point[0], point[1], point[2])
        packet = Ether() / Raw(load=payload)
        packets.append(packet)
    
    wrpcap(filename, packets)

def points_to_ply(points, filename):
    #test function
    with open(filename, 'w') as ply_file:
        # Write the header
        ply_file.write("ply\n")
        ply_file.write("format ascii 1.0\n")
        ply_file.write(f"element vertex {len(points)}\n")
        ply_file.write("property float x\n")
        ply_file.write("property float y\n")
        ply_file.write("property float z\n")
        ply_file.write("end_header\n")
        
        # Write the points
        for point in points:
            ply_file.write(f"{point[0]} {point[1]} {point[2]}\n")

def pcap_to_ply(pcap_filename, ply_filename):
 
    from scapy.all import rdpcap
    packets = rdpcap(pcap_filename)
    points = []

    for packet in packets:
    
        payload = packet[Raw].load
        x, y, z = struct.unpack('fff', payload[:12])
        points.append((x, y, z))

    points_to_ply(points, ply_filename)

def read_csv_to_points(csv_filename):
    """
    Reads a CSV file where columns 1, 2, and 3 are x, y, and z coordinates, respectively.

    :param csv_filename: The name of the CSV file.
    :return: A list of tuples, where each tuple contains three floats (x, y, z).
    """
    test=False
    points = []
    with open(csv_filename, 'r') as csv_file:
        csv_reader = csv.reader(csv_file)
        for row in csv_reader:
            #skip first row
            if test==False:
                test=True
                continue
            if len(row) >= 3:
                try:
                    x = float(row[1])
                    y = float(row[2])
                    z = float(row[3])
                    points.append((x, y, z))
                except ValueError as e:
                    print(f"Error converting row to floats: {row} - {e}")
    return points


if __name__ == '__main__':
    # Example usage:
    points = read_csv_to_points('output.csv')
    points_to_ply(points, 'output.ply')