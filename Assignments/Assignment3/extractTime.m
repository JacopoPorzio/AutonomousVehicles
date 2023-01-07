function time = extractTime(str, L)

time = zeros(L, 1);

for ii = 1 : L
    sec = cast(str{ii}.Header.Stamp.Sec, 'double');
    nsec = cast(str{ii}.Header.Stamp.Nsec, 'double');
    time(ii, 1) = sec + nsec*1e-9;
end

end
