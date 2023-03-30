function CreateFig(fignum)
% Function to create a figure (given number or if empty, new figure)
% with thicker lines, fonts, etc.
if nargin >0
    f=figure(fignum);
else
    f=figure;
end
set(f,'defaultLineLineWidth',2);
set(f,'defaultAxesLineWidth',1.5);
set(f,'defaultAxesFontSize',18);
set(f,'defaultAxesFontWeight','Bold');
set(f,'defaultTextFontSize',16);
set(f,'defaultTextFontWeight','Bold');
set(f,'defaultLineMarkerSize',10);