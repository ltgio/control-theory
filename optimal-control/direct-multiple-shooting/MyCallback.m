%
%     This file is part of CasADi.
%
%     CasADi -- A symbolic framework for dynamic optimization.
%     Copyright (C) 2010-2014 Joel Andersson, Joris Gillis, Moritz Diehl,
%                             K.U. Leuven. All rights reserved.
%     Copyright (C) 2011-2014 Greg Horn
%
%     CasADi is free software; you can redistribute it and/or
%     modify it under the terms of the GNU Lesser General Public
%     License as published by the Free Software Foundation; either
%     version 3 of the License, or (at your option) any later version.
%
%     CasADi is distributed in the hope that it will be useful,
%     but WITHOUT ANY WARRANTY; without even the implied warranty of
%     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
%     Lesser General Public License for more details.
%
%     You should have received a copy of the GNU Lesser General Public
%     License along with CasADi; if not, write to the Free Software
%     Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
%

classdef MyCallback < casadi.Callback
  properties
    nx
    ng
    np
    iter
  end
  methods
    function self = MyCallback(name, nx, ng, np)
      self@casadi.Callback();
      self.nx = nx;
      self.ng = ng;
      self.np = np;
      self.iter = 0;
      
      opts = struct;
      opts.input_scheme = casadi.nlpsol_out();
      opts.output_scheme = char('ret');
      construct(self, name, opts);
    end
    function out = get_sparsity_in(self,i)
      n = casadi.nlpsol_out(i);
      if strcmp(n,'f')
        out = [1 1];
      elseif strcmp(n,'lam_x') || strcmp(n,'x')
        out = [self.nx 1];
      elseif strcmp(n,'lam_g') || strcmp(n,'g')
        out = [self.ng 1];
      elseif strcmp(n,'p')  || strcmp(n,'lam_p')
        out = [self.np 1];
      else
        out = [0 0];
      end
      out = casadi.Sparsity.dense(out(1),out(2));
    end
    function out = get_n_in(self)
      out = casadi.nlpsol_n_out();
    end
    function out = get_n_out(self)
      out = 1;
    end

    % Evaluate numerically
    function out = eval(self, arg)
      global T N nx nu
      w = full(arg{1});
      %f = full(arg{2});
      y_opt       = w(1:nx+nu:end);
      z_opt       = w(2:nx+nu:end);
      theta_opt   = w(3:nx+nu:end);
      dy_opt      = w(4:nx+nu:end);
      dz_opt      = w(5:nx+nu:end);
      dtheta_opt  = w(6:nx+nu:end);

      T_opt       = w(7:nx+nu:end);
      F_opt       = w(8:nx+nu:end);

      time = linspace(0, T, N+1);
      
      clf;
      
      subplot(2,2,1);grid on;hold on;
      plot(time,y_opt    ,'r');
      plot(time,z_opt    ,'b');
      plot(time,theta_opt,'g');
      legend('y[t]','z[t]','\theta [t]');
      hold off

      subplot(2,2,2);grid on;hold on;
      plot(time,dy_opt    ,'r');
      plot(time,dz_opt    ,'b');
      plot(time,dtheta_opt,'g');
      legend('dy[t]','dz[t]','d \theta [t]');
      hold off

      subplot(2,2,3);grid on;hold on;
      plot(time,[T_opt;nan],'c');legend('T[t]');
      hold off
      subplot(2,2,4);grid on;hold on
      plot(time,[F_opt;nan],'m');legend('F[t]');
      hold off
      
%       self.iter = self.iter + 1;
     
      out = {0};
    end
  end
end
