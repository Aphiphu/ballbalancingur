x=[0,1,1,2,5,2,2,3,2,3,4,5]
y=x.copy()
for i in range(len(x)):
    if i==0: continue
    y[i]=y[i]-y[i-1]

print(y)