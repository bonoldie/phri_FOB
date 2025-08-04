function dtx = polydetrend(x,y)

p = polyfit(x,y,5)
f = polyval(p,x);

dtx = x-f;

figure(998)
plot(x,y,'o',x,f,'-')

