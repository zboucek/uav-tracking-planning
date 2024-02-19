function [fig] = plot_union_part(union,colors, edges)
%PLOT_UNION_PART plot partition of Union from MPT3
% INPUT:
%	Union - union for print
%   colors - matrix with RGB color in rows
%   edges - 0 = no edges, 1 = edges on
% OUTPUT:
%   fig - figure with union

colN = length(colors);

fig = figure;
hold on;
j = 1;

for i = 1:union.Num
    if union.Set(i).contains(zeros(union.Set(i).Dim,1))
        % set with origin has always same color (the first one)
        h = union.Set(i).plot('Color',colors(1,:));
        j = 1;
    else
        h = union.Set(i).plot('Color',colors(j,:));
    end
    if edges == 0
        % turn off edges
        set(h,'edgecolor','none');
    end
    j=j+1;
    if j > colN
        j = 1;
    end
end
hold off;

end