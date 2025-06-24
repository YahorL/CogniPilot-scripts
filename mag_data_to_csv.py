import csv

def extract_mag_to_csv(txtfile, csvfile):
    with open(txtfile) as fin, open(csvfile, 'w', newline='') as fout:
        writer = csv.writer(fout)
        #writer.writerow(['x', 'y', 'z'])  # header
        for line in fin:
            if line.strip().startswith('x:'):
                parts = line.strip().split()
                x = float(parts[1])
                y = float(parts[3])
                z = float(parts[5])
                writer.writerow([x, y, z])

extract_mag_to_csv('mag_data/mag_data.txt', 'mag_data/mag_data.csv')
