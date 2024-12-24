import datetime
import json

with open("C:\\Users\\a\\Documents\\GitHub\\Roki_2_Soccer\\record-2024-12-17.json" , "r") as f:
    data = json.loads(f.read())
print(data)
data_new ={}
data_len = len(data['pageNames'])
data_d = []
names_new = []
for i in range(data_len):
    data_d.append(data['record-2024-12-17'][data_len - 1 - i])
    names_new.append(data['pageNames'][data_len - 1 - i])
data_new ={'record-2024-12-17r': data_d, 'pageNames': names_new}

with open("C:\\Users\\a\\Documents\\GitHub\\Roki_2_Soccer\\record-2024-12-17r.json" , "w") as f:
    json.dump(data_new, f)