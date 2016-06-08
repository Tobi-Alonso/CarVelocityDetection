# codigo para graficar e interpolar linearmente el angulo minimo permitido en funcion del angulo maximo detectado (de vector normal a recta)

# x=2:0.1:7; 
# xth=0.5;
# theta=pi+atan(-x/1.3);
# theta_th= atan(-x/1.3)-atan(-(x+xth)/1.3);
# theta_min=theta-theta_th;
# plot(theta,theta_min);
# grid;


x=-2:-0.1:-7; 
xth=0.5;
theta=atan(-x/1.3);
theta_th= atan(-(x+xth)/1.3)-atan(-x/1.3);
theta_max=theta+theta_th;

theta_int=1.3143*theta-0.44543;
plot(theta,theta_max,'r');
grid;

hold;
plot(theta,theta_int,'b');
