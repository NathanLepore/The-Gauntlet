function [Obj] = pcalc(x, y, xfinal, yfinal)
Obj = (1./sqrt(((x - xfinal).^2+(y - yfinal).^2)));
