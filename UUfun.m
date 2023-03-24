function U = UUfun(RR,r_x,r_y,wpx,wpy)
%UUfun
%    U = UUfun(RR,R_X,R_Y,WPX,WPY)

%    This function was generated by the Symbolic Math Toolbox version 9.0.
%    24-Mar-2023 09:38:05

U = exp(-sqrt(abs(r_x-wpx).^2+abs(r_y-wpy).^2)./RR);
