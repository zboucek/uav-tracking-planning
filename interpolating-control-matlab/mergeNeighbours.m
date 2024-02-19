function [Parray] = mergeNeighbours(Parray)
%MERGENEIGHBOURS Find neigboring and containded set of Parray and merge
%them

i = 1;
while i <= length(Parray)
    j = 1;
    while j <= length(Parray)
        if Parray(i).isNeighbor(Parray(j))
            % if are neighbors, merge, add merged and erase old
            Parray(end+1) = Polyhedron([Parray(i).V;Parray(j).V]);
            Parray([i,j]) = [];
            i = 1; j = 1;
        end
        if Parray(i).contains(Parray(j)) && i ~= j
            % if is contained in another set, erase
        	Parray(j) = [];
            i = 1; j = 1;
        end
        j = j+1;
    end
    i = i+1;
end