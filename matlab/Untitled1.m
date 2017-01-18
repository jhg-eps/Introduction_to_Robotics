clear
n=100;
m=zeros(n,n);
for i=1:n-1
    alpha(i)=1; %0.9^(i-1);
    m(i:i+1,i:i+1)=m(i:i+1,i:i+1)+[-1 1;1 -1].*alpha(i);
end
m(1,1)=-2;
m(n,n)=-2;
[v,w2]=eig(m);




plot_eigenvectors=1;
plot_eigenvalues=1;

if plot_eigenvalues==1
    for i=1:n
        x(i)=i;
        y(i)=w2(i,i);
    end
    logy=log(-y);
    figure
    plot(x,y,'.');
    xlabel('Mode Number');
    ylabel('Negative Frequency Squared');
end
colors=lines(n);
if plot_eigenvectors==1
    figure
    hold on
    x=1:n;
    curves_to_plot=[1,99,100];
 %   curves_to_plot=[97,98,99,100];

    ncurves=length(curves_to_plot);
    for j=1:ncurves
        y=v(:,curves_to_plot(j));
        plot(x,y,'.-','Color',colors(curves_to_plot(j),:));
        legend_text{j}=strcat('mode=',num2str(curves_to_plot(j)));

    end
    legend(legend_text,'Location','Best');
    xlabel('Mass Number');
    ylabel('Amplitude');

end
%ylim([-0.5 0.5]);
