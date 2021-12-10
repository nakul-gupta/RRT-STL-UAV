%% Scatters

zero = histogram(robustness_matrix(:,1), 25)
handleOfHistogramFigure_zero = ancestor(zero, 'figure');
figure(handleOfHistogramFigure_zero);
one = histogram(robustness_matrix(:,2), 25)
handleOfHistogramFigure_one = ancestor(one, 'figure');
figure(handleOfHistogramFigure_one);
two = histogram(robustness_matrix(:,3), 25)
handleOfHistogramFigure_two = ancestor(two, 'figure');
figure(handleOfHistogramFigure_two);
three = histogram(robustness_matrix(:,4), 25)
handleOfHistogramFigure_three = ancestor(three, 'figure');
figure(handleOfHistogramFigure_zero);
four = histogram(robustness_matrix(:,5), 25)
handleOfHistogramFigure_four = ancestor(four, 'figure');
figure(handleOfHistogramFigure_four);
% % Obtain the handle of the figure that contains the histogram
% handleOfHistogramFigure = ancestor(zero, 'figure');
% % Make the figure window visible in case it was invisible before
% handleOfHistogramFigure.Visible  = 'on'
% % Bring the figure window to the front
% figure(handleOfHistogramFigure);



% x = randn(10000,1);
% h = histogram(x)