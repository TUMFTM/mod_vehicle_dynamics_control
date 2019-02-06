function s_out = struct2sims(s_in, name)
    s_out = struct();
    f = fieldnames(s_in);
    for ii = 1:numel(f)
        if(isempty(name))
          subname = f{ii};
        else
          subname = [name '_' f{ii}];
        end
        val = s_in.(f{ii});
        if isstruct(val)
            s_tmp = struct2sims(val, subname);
            ff = fieldnames(s_tmp);
            for jj = 1:numel(ff)
                s_out.(ff{jj}) = s_tmp.(ff{jj});
            end
        else
            s_out.(subname) = val;
        end
    end
end