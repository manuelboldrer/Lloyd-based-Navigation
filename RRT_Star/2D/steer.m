function A = steer(qr, qn, val, eps)
   qnew = [0 0];
   
   % Steer towards qn with maximum step size of eps
   if val >= eps
       qnew(1) = qn(1) + ((qr(1)-qn(1))*eps)/dist1(qr,qn);
       qnew(2) = qn(2) + ((qr(2)-qn(2))*eps)/dist1(qr,qn);
   else
       qnew(1) = qr(1);
       qnew(2) = qr(2);
   end   
   A = [qnew(1), qnew(2)];
end