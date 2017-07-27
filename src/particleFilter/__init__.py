import numpy as np
from numpy import zeros, ones, sqrt
import scipy
import scipy.stats


from random import random
from bisect import bisect


def weighted_choice(weights, values):
    total = 0
    cum_weights = []
    for w in weights:
        total += w
        cum_weights.append(total)
    x = random() * total
    i = bisect(cum_weights, x)
    return values[i]


def npdf(x, mu, sigma):
    return scipy.stats.multivariate_normal(mu, sigma).pdf(x)


def nsamp(mu, sigma):
    return np.random.multivariate_normal(mu, sigma)


class ParticleFilter(object):
    
    def __init__(self):
        s = self
        s.nx = 1  # number of states
        s.ny = 1  # number of observables
        s.nu = 1  # size of process noise vector
        s.nv = 1  # size of observation noise vector
        
        # system/process noise PDF
        s.mu_u = [0] * s.nu
        s.sigma_u = np.eye(s.nu) * sqrt(10)
        
        # observation noise PDF
        s.mu_v = [0] * s.nu
        s.sigma_v = np.eye(s.nu) * 1
        
        # initial PDF
        s.mu_initial = [0] * s.nx
        s.sigma_initial = np.eye(s.nx) * sqrt(10) 
        
    def sys(self, k, xkm1, uk):
        '''Process equation'''
        return xkm1 * .5 + 26 * xkm1 / (1 + xkm1**2) + 8 * np.cos(1.2 * k) + uk
    
    def obs(self, k, xk, vk):
        '''Observation equation'''
        vk = np.asarray(vk)
        return xk**2 / 20 + vk
    
    def p_sys_noise(self, u):
        return npdf(u, self.mu_u, self.sigma_u)
    def gen_sys_noise(self):
        return nsamp(self.mu_u, self.sigma_u)
        
    def p_obs_noise(self, v):
        return npdf(v, self.mu_v, self.sigma_v)
    def gen_obs_noise(self):
        return nsamp(self.mu_v, self.sigma_v)
    
#     def p_x0(self, x):
#         return npdf(x, self.mu_initial, self.sigma_initial)
    def gen_x0(self):
        return nsamp(self.mu_initial, self.sigma_initial)
    
#     def p_xk_given_xkm1(self, k, xk, xkm1):
#         return self.p_sys_noise(xk - self.sys(k, xkm1, [0]*self.nu))
    
    def p_yk_given_xk(self, k, yk, xk):
        return self.p_obs_noise(yk - self.obs(k, xk, [0]*self.nv))
    
    def simulate(self, T=40):
        s = self
        x = zeros((s.nx, T))
        y = zeros((s.ny, T))
        u = zeros((s.nu, T))
        v = zeros((s.nv, T))
        
        # Simulate system
        xh0 = 0                                  # initial state
        u[:, 0] = 0                               # initial process noise
        v[:, 0] = s.gen_obs_noise()          # initial observation noise
        x[:, 0] = xh0
        y[:, 0] = s.obs(0, xh0, v[:, 0])
        for k in range(1, T):
            # here we are basically sampling from p_xk_given_xkm1 and from p_yk_given_xk
            u[:, k] = s.gen_sys_noise()              # simulate process noise
            v[:, k] = s.gen_obs_noise()              # simulate observation noise
            x[:, k] = s.sys(k, x[:, k-1], u[:, k])     # simulate state
            y[:, k] = s.obs(k, x[:, k],   v[:, k])     # simulate observation
        
        ## Separate memory
        xh = zeros((s.nx, T))
        xh[:,1] = xh0
        yh = zeros((s.ny, T))
        yh[:,1] = s.obs(1, xh0, [0]*s.nv)
        pf = dict(
            k               = 1                      ,# initial iteration number
            Ns              = 200                    ,# number of particles
            w               = zeros((200, T))        ,# weights
            particles       = zeros((s.nx, 200, T))  ,# particles
            gen_x0          = s.gen_x0               ,# function for sampling from initial pdf p_x0
            p_yk_given_xk   = s.p_yk_given_xk        ,# function of the observation likelihood PDF p(y[k] | x[k])
            gen_sys_noise   = s.gen_sys_noise        ,# function for generating system noise
            #p_x0 = p_x0                             ,# initial prior PDF p(x[0])
            #p_xk_given_ xkm1 = p_xk_given_xkm1      ,# transition prior PDF p(x[k] | x[k-1])
        )
        
        
        ## Estimate state
        for k in range(1, T):
            print('Iteration = %d/%d' %(k, T))
            # state estimation
            pf['k'] = k
            #[xh[:, k], pf] = particle_filter(sys, y[:, k], pf, 'multinomial_resampling')
            xh[:, k], pf = s.do_filter(y[:, k], pf, 'systematic_resampling')   
            #[xh[:, k], pf] = particle_filter(sys, y[:, k], pf, 'regularized_pf')   
           
            # filtered observation
            yh[:, k] = s.obs(k, xh[:, k], 0)
        
        return x, y, u, v, pf
        
    def do_filter(self, yk, pf, resampling_strategy):
        k = pf['k']
        if k == 0:
            raise ValueError('k must be an integer greater or equal than 1.')
                
        ## Initialize variables
        Ns = pf['Ns']                              # number of particles
        nx = pf['particles'].shape[0]               # number of states
        
        wkm1 = pf['w'][:, k-1]                   # weights of last iteration
        if k == 1:
            for i in range(Ns):                          # simulate initial particles
                pf['particles'][:, i, 0] = pf['gen_x0']() # at time k=1
            wkm1 = ones((Ns, 1)) * 1./Ns           # all particles have the same weight
        
        ##
        # The importance sampling function:
        # PRIOR: (this method is sensitive to outliers)   THIS IS THE ONE USED HERE
        # q_xk_given_xkm1_yk = pf.p_xk_given_xkm1
        
        # OPTIMAL:
        # q_xk_given_xkm1_yk = q_xk_given_xkm1^i_yk
        # Note this PDF can be approximated by MCMC methods: they are expensive but 
        # they may be useful when non-iterative schemes fail
        
        ## Separate memory
        xkm1 = pf['particles'][:, :, k-1] # extract particles from last iteration
        xk   = zeros((xkm1.shape))     # = zeros((nx,Ns)
        wk   = zeros((wkm1.shape))     # = zeros((Ns,1)
        
        ## Algorithm 3 of Ref [1]
        for i in range(Ns):
            # xk(:,i) = sample_vector_from q_xk_given_xkm1_yk given xkm1(:,i) and yk
            # Using the PRIOR PDF: pf.p_xk_given_xkm1: eq 62, Ref 1.
            xk[:, i] = self.sys(k, xkm1[:, i], pf['gen_sys_noise']())
            
            # Equation 48, Ref 1.
            # wk(i) = wkm1(i) * p_yk_given_xk(yk, xk(:,i))*p_xk_given_xkm1(xk(:,i), xkm1(:,i))/q_xk_given_xkm1_yk(xk(:,i), xkm1(:,i), yk)
            
            # weights (when using the PRIOR pdf): eq 63, Ref 1
            wk[i] = wkm1[i] * pf['p_yk_given_xk'](k, yk, xk[:, i])
            
            # weights (when using the OPTIMAL pdf): eq 53, Ref 1
            # wk(i) = wkm1(i) * p_yk_given_xkm1(yk, xkm1(:,i)) # we do not know this PDF
        
        ## Normalize weight vector
        wk /= sum(wk)
        
        ## Calculate effective sample size: eq 48, Ref 1
        Neff = 1. / sum(wk**2)
        
        ## Resampling
        # remove this condition and sample on each iteration:
        # [xk, wk] = resample(xk, wk, resampling_strategy)
        #if you want to implement the bootstrap particle filter
        resample_percentaje = 0.50
        Nt = resample_percentaje*Ns
        if Neff < Nt:
            print('Resampling ...')
            xk, wk, _idx = self.resample(xk, wk, resampling_strategy)
            # {xk, wk} is an approximate discrete representation of p(x_k | y_{1:k})
        
        ## Compute estimated state
        xhk = zeros((nx,1))
        for i in range(Ns):
            xhk = xhk + wk.ravel()[i] * xk[:, i]
        
        ## Store new weights and particles
        pf['w'][:,k] = wk
        pf['particles'][:, :, k] = xk
        
        return xhk, pf
    
    def resample(self, xk, wk, resampling_strategy):    
        ## Resampling function
#         function [xk, wk, idx] = resample(xk, wk, resampling_strategy)
        
        Ns = len(wk)  # Ns = number of particles
        
        # wk = wk./sum(wk) # normalize weight vector (already done)
        
        if resampling_strategy == 'multinomial_resampling':
            choices = range(Ns)
            idx = [weighted_choice(wk, choices) for _ in range(Ns)]
            '''
            THIS IS EQUIVALENT TO:
            edges = min([0 cumsum(wk)'],1) # protect against accumulated round-off
            edges(end) = 1                 # get the upper edge exact
            # this works like the inverse of the empirical distribution and returns
            # the interval where the sample is to be found
            [~, idx] = histc(sort(rand(Ns,1)), edges)
            '''
        elif resampling_strategy == 'systematic_resampling':
            # this is performing latin hypercube sampling on wk
            edges = np.minimum(
                np.hstack((0, np.cumsum(wk))),
                1
            )  # protect against accumulated round-off
            edges[-1] = 1                 # get the upper edge exact
            u1 = np.random.rand() / Ns
            # this works like the inverse of the empirical distribution and returns
            # the interval where the sample is to be found
            idx = np.digitize(np.arange(u1, 1, 1./Ns), edges) - 1
           
#         case 'regularized_pf' 
#         
#                 #reasmple 
#                 edges = min([0 cumsum(wk)'],1) # protect against accumulated round-off 
#                 edges(end) = 1 # get the upper edge exact 
#                 u1 = rand/Ns 
#         
#                 [~, idx] = histc(u1:1/Ns:1, edges) 
#                 xk = xk(:,idx) # extract new particles 
#                 wk = repmat(1/Ns, 1, Ns) # now all particles have the same weight 
#         
#                 #according to Mussao et al., 2001 
#                 # compute empirical covariance of particles 
#                 emp_cov=cov(xk')' 
#                 # form D'*D=emp_cov 
#                 dd=cholcov(emp_cov) 
#         
#                 nx=size(xk,1) 
#                 #unit sphere volume (in two dimensions) 
#                 cc=pi 
#                 #form the optimal choice of bandwidth 
#                 aa=(8*(1/cc)*(nx+4)*(2*pi^.5)^nx)^(1/(nx+4)) 
#                 hopt=aa*Ns^(-1/(nx+4)) 
#                 # form an estimation of continuous pdf via epanechnikov kernel 
#                 [f,~] = ksdensity(wk,'npoints',length(wk),'kernel','epanechnikov') 
#                 f=f/sum(f) 
#                 # compute the cumulative of the continuous distribution 
#                 edges = min([0 cumsum(f)],1) # protect against accumulated round-off 
#                 edges(end) = 1 # get the upper edge exact 
#                 #sample from the inverse of cumulative of continuous density 
#                 u1 = rand/Ns 
#                 [~, idx] = histc(u1:1/Ns:1, edges) 
#                 ee=xk(:,idx) 
#                 #move all samples to centre 
#                 ee=ee-repmat(mean(ee,2),1,length(ee)) 
#                 # adjust resampled particles 
#                 xk = xk+hopt*dd*ee # extract new particles 
#                 wk = repmat(1/Ns, 1, Ns)
#               
#            # case 'stratified_sampling' TO BE IMPLEMENTED
#            # case 'residual_sampling'   TO BE IMPLEMENTED
#            otherwise
#               error('Resampling strategy not implemented')
#         end
        
        xk = xk[:, idx]                    # extract new particles
        wk = ones((1, Ns)) * 1. / Ns         # now all particles have the same weight
        
        return xk, wk, idx  


if __name__ == '__main__':
    filter = ParticleFilter()
    x, y, u, v, pf = filter.simulate(42)
    
    print(x.shape, y.shape, u.shape, v.shape)
    print(pf.keys())
    particles = pf['particles']
    print(particles.shape)
    
    
    import matplotlib.pyplot as plt
    
    fig, ax = plt.subplots()
    ax.plot(x.ravel(), label='x', color='black', lw=4)
    for i in range(pf['Ns']):
        if i == 0:
            label = 'particles'
        else:
            label = None
        ax.plot(particles[:, i, :].ravel(), label=label, color='blue', alpha=.1)
    ax.legend(loc='best')
    plt.show()