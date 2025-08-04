function v_pos = positive(v)
v_pos = v;

for i=1:length(v)
    if(v(i)<0)
        v_pos(i)=0;
    end
end