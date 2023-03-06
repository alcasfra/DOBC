function [dw] = dw(t)

    global a0 b0 a1 b1 a2 b2 w0 w1 w2

    dw = a0*w0*cos(w0*t) + a1*w1*cos(w1*t) + a2*w2*cos(w2*t) - ...
        b0*w0*sin(w0*t) - b1*w1*sin(w1*t) - b2*w2*sin(w2*t);
    
end