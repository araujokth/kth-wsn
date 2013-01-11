PATH = 'test_5'

MOTE2 = dlmread(strcat(PATH, '/Mote2_R.txt'))
MOTE3 = dlmread(strcat(PATH, '/Mote3_R.txt'))
MOTE4 = dlmread(strcat(PATH, '/Mote4_R.txt'))
MOTE5 = dlmread(strcat(PATH, '/Mote5_R.txt'))

MOTE6 = dlmread(strcat(PATH, '/Mote6_S.txt'))
MOTE7 = dlmread(strcat(PATH, '/Mote7_S.txt'))
MOTE8 = dlmread(strcat(PATH, '/Mote8_S.txt'))
MOTE9 = dlmread(strcat(PATH, '/Mote9_S.txt'))

TIMESTAMP = dlmread(strcat(PATH, '/Time_reRouting.txt'))
% 3 rows: 1st,2nd,3rd breaking & rerouting time
t = reshape(TIMESTAMP.',1,6)

pkts_cnt = zeros(8, 6) % column 1,3,5: BEACON, column 2,4,6: DATA

% Relay motes stop after it/link is broken
% Mote 5,4,3 send no packet after 1st/2nd/3rd link breaking, respectively

% Mote 5: 0 beacon, 0 data during all three rerouting time

% Mote 4: 0 beacon, 0 data during the 2nd & 3rd rerouting time
for i = 1:size(MOTE4,1)
    if MOTE4(i,1) >= t(1)
        if MOTE4(i,1) <= t(2)
            index = MOTE4(i,2) / 111
            pkts_cnt(3,index) = pkts_cnt(3,index) + 1
        else
            break
        end
    end
end

% Mote 3: 0 beacon, 0 data during the 3rd rerouting time
for i = 1:size(MOTE3,1)
    index = MOTE3(i,2) / 111
    if MOTE3(i,1) >= t(1)
        if MOTE3(i,1) <= t(2)
            pkts_cnt(2,index) = pkts_cnt(2,index) + 1
        elseif MOTE3(i,1) >= t(3)
            if MOTE3(i,1) <= t(4)
                index = index + 2
                pkts_cnt(2,index) = pkts_cnt(2,index) + 1
            else
                break
            end
        end
    end
end

% Mote 2 and 6~9 are not broken
for i = 1:size(MOTE2,1)
    if MOTE2(i,1) >= t(1) & MOTE2(i,1) <= t(6)
        index = counting(MOTE2(i,1),MOTE2(i,2),t(1),t(2),t(3),t(4),t(5),t(6)) 
        if index > 0
            pkts_cnt(1,index) = pkts_cnt(1,index) + 1
        end
    end
end

for i = 1:size(MOTE6,1)
    if MOTE6(i,1) >= t(1) & MOTE6(i,1) <= t(6)
        index = counting(MOTE6(i,1),MOTE6(i,2),t(1),t(2),t(3),t(4),t(5),t(6)) 
        if index > 0
            pkts_cnt(5,index) = pkts_cnt(5,index) + 1
        end
    end
end

for i = 1:size(MOTE7,1)
    if MOTE7(i,1) >= t(1) & MOTE7(i,1) <= t(6)
        index = counting(MOTE7(i,1),MOTE7(i,2),t(1),t(2),t(3),t(4),t(5),t(6)) 
        if index > 0
            pkts_cnt(6,index) = pkts_cnt(6,index) + 1
        end
    end
end

for i = 1:size(MOTE8,1)
    if MOTE8(i,1) >= t(1) & MOTE8(i,1) <= t(6)
        index = counting(MOTE8(i,1),MOTE8(i,2),t(1),t(2),t(3),t(4),t(5),t(6)) 
        if index > 0
            pkts_cnt(7,index) = pkts_cnt(7,index) + 1
        end
    end
end

for i = 1:size(MOTE9,1)
    if MOTE9(i,1) >= t(1) & MOTE9(i,1) <= t(6)
        index = counting(MOTE9(i,1),MOTE9(i,2),t(1),t(2),t(3),t(4),t(5),t(6)) 
        if index > 0
            pkts_cnt(8,index) = pkts_cnt(8,index) + 1
        end
    end
end

dlmwrite(strcat(PATH, '/packet_count_during_rerouting.txt'), pkts_cnt, 'delimiter', ' ', 'precision', '%d');