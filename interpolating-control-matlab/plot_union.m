function [fig] = plot_union(union,colors,edges)
% plot function of Union from MPT3
% INPUT:
%	Union - union for print
%   colors - matrix with RGB color in rows
%   edges - 0 = no edges, 1 = edges on
% OUTPUT:
%   fig - figure with union


colN = length(colors);
funN = length(union.listFunctions);

fig = figure;
hold on;
j = 1;
for i = 1:union.Num
    if funN > 1
        % check because IC has more functions
        h = union.Set(i).fplot('u','Color',colors(j,:));
    else
        if union.Set(i).contains(zeros(union.Set(i).Dim,1))
        % set with origin has always same color (the first one)
            h = union.Set(i).fplot('Color',colors(1,:));
            j = 1;
        else
            h = union.Set(i).fplot('Color',colors(j,:));
        end
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

