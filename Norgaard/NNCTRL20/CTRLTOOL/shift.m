function out_vector = shift(in_vector, new_element)
% Shifts all elements of a vector and inserts a new value 
% in the first element. The last element is lost.
%
% Inputs:   in_vector   : Vector to be shifted
%           new_element : New element to be inserted
%
% Outputs:  out_vector  : The updated vector

out_vector = [new_element ; in_vector(1:length(in_vector)-1)];

