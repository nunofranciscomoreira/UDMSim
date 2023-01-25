import glob
import xlwt


outputs = []  # type: List[str]
workbook = xlwt.Workbook()
sheet1 = workbook.add_sheet("N Data Mules LOGS CRAS")
for file0 in sorted(glob.glob("*.txt")):
    with open(file0) as f:
        content = f.readlines()  # content = 'PFT-LBL:1-30:100-200:100-500:Rb'
        content = [x.strip() for x in content]
    # outputs = ['1-30:100-200:100-500:Rb', '1-30:100-200:100-500:Rb' ...]
    # tipo << ":" << seed << ":" << nmules << ":" << distance << ":" << Rb <<
    outputs.append(content)

# print(outputs)

sheet1.write(0, 0, 'Type')
sheet1.write(0, 1, 'Seed')
sheet1.write(0, 2, 'Mules')
sheet1.write(0, 3, 'Distance')
sheet1.write(0, 4, 'bit/s')
sheet1.write(0, 5, 'Datasize')
sheet1.write(0, 6, 'Phy_type')
sheet1.write(0, 7, 'BandWidth')
i = 0
j = 1
for output in outputs:
    toWrite = output[0].split(":")
    while i < len(toWrite):
        sheet1.write(j, i, toWrite[i])
        i += 1
    i = 0
    j += 1

workbook.save('parsed.xls')
