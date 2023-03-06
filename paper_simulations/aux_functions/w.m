function [w] = w(t)
    
    global a0 b0 a1 b1 a2 b2 w0 w1 w2

    w = a0*sin(w0*t) + a1*sin(w1*t) + a2*sin(w2*t) + ...
        b0*cos(w0*t) + b1*cos(w1*t) + b2*cos(w2*t);
    
end