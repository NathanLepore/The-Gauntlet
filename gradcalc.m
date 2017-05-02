function [grad] = gradcalc(x1, y1, xfinal, yfinal)
step = 0.0001;
grad = [(pcalc(x1 + step,y1, xfinal, yfinal) - pcalc(x1 - step, y1,xfinal, yfinal))/(step*2);(pcalc(x1, y1 + step,xfinal, yfinal) - pcalc(x1,y1 - step,xfinal, yfinal))/(step*2)];
