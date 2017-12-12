import matplotlib.pyplot as plt
x=[]
y=[]
y2=[]


data = [line.strip().split(";") for line in open('measure6.csv')]
labels = data[0]
data = data[1:]
data = [map(float, line) for line in data if "None" not in line]
data = list(zip(*data)) # time, global_x, global_y, global_yaw, relative_x, relative_y, relative_yaw
#print(data)

# plot curve
plt.xlabel(labels[1])
plt.ylabel(labels[2])
plt.plot(data[1], data[2], "oy",)
plt.show()
plt.xlabel(labels[4])
plt.ylabel(labels[5])
plt.plot(data[4], data[5], "ob",)
plt.show()

# plot x
plt.xlabel(labels[0])
plt.ylabel(labels[1]+";"+labels[4])
plt.plot(data[0], data[1], "oy",)
plt.plot(data[0], data[4], "ob",)
plt.show()

# plot y
plt.xlabel(labels[0])
plt.ylabel(labels[2]+";"+labels[5])
plt.plot(data[0], data[2], "oy",)
plt.plot(data[0], data[5], "ob",)
plt.show()

# plot yaw
plt.xlabel(labels[0])
plt.ylabel(labels[3]+";"+labels[6])
plt.plot(data[0], data[3], "oy",)
plt.plot(data[0], data[6], "ob",)
plt.show()



"""
for line in fobj:
    h=line.strip()
    x.append(h.split(';')[0])
    y.append(h.split(';')[1])
    y2.append(h.split(';')[2])
fobj.close()
xname=x[0]
yname=y[0]+', '+y2[0]
plt.xlabel(xname)
plt.ylabel(yname)
x=list(map(float,x[1:]))
y=list(map(float,y[1:]))
y2=list(map(float,y2[1:]))
plt.plot(x, y, "y",)
plt.plot(x, y2, "b",)
plt.show()
"""