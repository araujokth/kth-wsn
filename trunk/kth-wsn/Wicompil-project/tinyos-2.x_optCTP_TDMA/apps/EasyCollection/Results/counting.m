function flag = counting(val1, val2, t1, t2, t3, t4, t5, t6)
    flag = 0
    index = val2 / 111
    if val1 >= t1
        if val1 <= t2
            flag = index
        elseif val1 >= t3
            if val1 <= t4
                flag = index + 2
            elseif val1 >= t5
                if val1 <= t6
                    flag = index + 4
                end
            end
        end
    end
end