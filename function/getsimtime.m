function [t_start, t_end] = getsimtime(UTC_time, tG)

% get simulation start and end time in UTC time

% get GPS time
UTC_num = datenum(UTC_time);

% GPS start time [UTC]
t_start = datevec(addtodate(UTC_num, -tG, 'sec'));

% GPS end time [UTC]
t_end = datevec(addtodate(UTC_num, 0, 'sec'));

end