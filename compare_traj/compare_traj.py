import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.pylab as pylab
import numpy as np
import argparse


if __name__ == '__main__':
        parser = argparse.ArgumentParser(description='''
        draw rpe for direct and indirect together
        ''')
        parser.add_argument('indirect_rpe', help='format: [stamp, trans_error]')
        parser.add_argument('direct_rpe', help='format: [stamp, trans_error]')
        args = parser.parse_args()

        indi = np.load(args.indirect_rpe)
        di = np.load(args.direct_rpe)

        stamp = indi[:,0]
        indirect_rpe = indi[:,1]
        direct_rpe = di[:,1]
        
        

        fig = plt.figure()
        ax = fig.add_subplot(111)        
        ax.plot(stamp,indirect_rpe,'-',color="blue", label="indirect")
        ax.plot(stamp,direct_rpe,'-',color="red", label="direct")
        #ax.plot([t for t,e in err_rot],[e for t,e in err_rot],'-',color="red")
        ax.legend()
        ax.set_xlabel('time [s]')
        ax.set_ylabel('translational error [m]')
        plt.savefig("fig.png",dpi=300)