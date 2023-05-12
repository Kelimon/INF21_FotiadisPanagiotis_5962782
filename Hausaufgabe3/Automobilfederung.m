%function [tsimout, ysimout] = function_name(c1, c2, d2, m1, m2, u)
%FUNCTION_NAME - This function implements a car suspension model and 
%                simulates its behavior over time using the provided 
%                parameters. The simulation results are displayed graphically.
%
% Outputs:
%    tsimout - Simulation time (x-Axis Values)
%    ysimout - Simulation results (y-Axis Values)
%
% Required Inputs:
%    c1, c2 - Damping coefficients
%    d2 - Car body mass
%    m1, m2 - Car wheel and body masses
%    u - External force as a function handle
%
% Other m-files required: none
% Subfunctions: none
% MAT-files required: none
%
% Example: 
%    [t, y] = function_name(500, 700, 1500, 2500, 320, @(t) 0.5*sin(t));
%
% Author: Panagiotis Fotiadis
% Email: inf21215@lehre.dhbw-stuttgart.de
% GitHub: https://github.com/Kelimon/INF21_FotiadisPanagiotis_5962782
% Date: 12.05.2023;
%
%------------- BEGIN CODE --------------
classdef Automobilfederung < handle
    properties
        c1 {mustBeNumeric}
        c2 {mustBeNumeric}
        d2 {mustBeNumeric}
        m1 {mustBeNumeric}
        m2 {mustBeNumeric}
        u
        A {mustBeNumeric}
        B {mustBeNumeric}
        tsimout {mustBeNumeric}
        ysimout {mustBeNumeric}
    end
    methods (Access = public)
        function obj = Automobilfederung(varargin)
            for i = 1:2:nargin
                if strcmp(varargin{i},'c1')
                    obj.c1 = varargin{i+1};
                elseif strcmp(varargin{i},'c2')
                    obj.c2 = varargin{i+1};
                elseif strcmp(varargin{i},'d2')
                    obj.d2 = varargin{i+1};
                elseif strcmp(varargin{i},'m1')
                    obj.m1 = varargin{i+1};
                elseif strcmp(varargin{i},'m2')
                    obj.m2 = varargin{i+1};
                elseif strcmp(varargin{i},'u')
                    if isa(varargin{i+1},'function_handle')
                        obj.u = varargin{i+1};
                    else
                        error("u seems not to be a function handle");
                    end
                else
                    warning("Invalid property: "+varargin{i});
                end
            end
            obj.calcSystemMartixA();
            obj.calcInputMatixB();
        end
        function sim(obj, varargin)
            t = 0;
            tfinal = 10;
            h = 0.01;
            y = [0; 0; 0; 0];
            for i = 1:2:nargin-1
                % ========= YOUR CODE HERE =========
                % perform the varargin: overwrite the defaults
                if strcmp(varargin{i}, 't0')
                    t = varargin{i + 1};
                elseif strcmp(varargin{i}, 'tfinal')
                    tfinal = varargin{i + 1};
                elseif strcmp(varargin{i}, 'y0')
                    y = varargin{i + 1};    % no y=y(:) operation, every matrix is transposed
                elseif strcmp(varargin{i}, 'stepsize')
                    h = varargin{i + 1};
                else
                    warning("Invalid property: "+varargin{i});
                end
            end
            tout = zeros(ceil((tfinal-t)/h)+1,1);
            yout = zeros(ceil((tfinal-t)/h)+1,length(y));
            tout(1) = t;
            yout(1,:) = y';
            step = 1;
            while (t < tfinal)
                step = step + 1;
                if t + h > tfinal
                    % ========= YOUR CODE HERE =========
                    % calculate h
                    h = tfinal - t; 
                end
                % ========= YOUR CODE HERE =========
                % calculate the slopes
                slope1 = rhs(obj, t, y);  
                slope2 = rhs(obj, t + 0.5 * h, y + 0.5 * h * slope1'); 
                slope3 = rhs(obj, t + 0.5 * h, y + 0.5 * h * slope2'); 
                slope4 = rhs(obj, t + h, y + h * slope3');
                
                % Update y using the calculated slopes
                ynew = y + (h / 6) * (slope1' + 2 * slope2' + 2 * slope3' + slope4'); 

                
                t = t + h;
                y = ynew;
                tout(step) = t;
                yout(step,:) = y';
                obj.tsimout = tout;
                obj.ysimout = yout;
            end
        end
        function fig = visualizeResults(obj)
            fig = figure('Name','Ergebnisse der Simulation');
            subplot(2,1,1);
            plot(obj.tsimout,obj.ysimout(:,1),'s-',...
                 obj.tsimout,obj.ysimout(:,3),'x-')
            grid on;
            ylabel('Höhe in m');
            legend('Karosserie','Rad');
            title("Position der Zustände | stepsize = "+num2str(obj.tsimout(2)-obj.tsimout(1)))
            subplot(2,1,2);
            plot(obj.tsimout,obj.ysimout(:,2),'s-',...
                 obj.tsimout,obj.ysimout(:,4),'x-')
            grid on;
            ylabel('Geschwindigkeit in m/s');
            xlabel('Simulationszeit in s');
            legend('Karosserie','Rad');
            title("Geschwindigkeit der Zustände | stepsize = "+num2str(obj.tsimout(2)-obj.tsimout(1)))
        end
    end
    methods (Access = private)
        function calcInputMatixB(obj)
            % ========= YOUR CODE HERE =========
            % Calculate input matrix B
            obj.B = [0; 0; 0; obj.c1/obj.m1];
        end
        function calcSystemMartixA(obj)
            % ========= YOUR CODE HERE =========
            % Compute system matrix A
            obj.A = [0, 1, 0, 0; -obj.c2/obj.m2, -obj.d2/obj.m2, obj.c2/obj.m2, obj.d2/obj.m2;...
                    0, 0, 0, 1; obj.c2/obj.m1, obj.d2/obj.m1,-(obj.c1+obj.c2)/obj.m1, -obj.d2/obj.m1];
        end
        function xdot = rhs(obj, t, x)
            x = x(:);
            xdot = obj.A*x + obj.B*obj.u(t);
        end
    end
end