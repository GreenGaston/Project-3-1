from scapy.all import wrpcap, Ether, Raw
import struct

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

if __name__ == '__main__':
    # Example usage:
    points = [(1.0, 2.0, 3.0), (4.0, 5.0, 6.0), (7.0, 8.0, 9.0)]
    points_to_pcap(points, 'input.pcap')
    pcap_filename = 'input.pcap'
    ply_filename = 'output.ply'
    pcap_to_ply(pcap_filename, ply_filename)