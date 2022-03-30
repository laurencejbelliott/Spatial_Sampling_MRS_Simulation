# -*- coding: utf-8 -*-
# Code extended to a module, originally from
# https://geostat-framework.readthedocs.io/projects/pykrige/en/latest/examples/08_krige_cv.html#sphx-glr-examples-08-krige-cv-py
"""
Krige CV
--------

Searching for optimal kriging parameters with cross validation
"""

import numpy as np
from pykrige.rk import Krige
from sklearn.model_selection import GridSearchCV


# 2D Kring param opt

param_dict = {
    # "method": ["ordinary", "universal"],
    "method": ["ordinary"],
    "variogram_model": ["linear", "power", "gaussian", "spherical"],
    # "nlags": [4, 6, 8],
    # "weight": [True, False]
}

estimator = GridSearchCV(Krige(), param_dict, verbose=True, return_train_score=True)


def kriging_param_cv(x, y):
    # # dummy data
    X = np.array(x, dtype=np.float64)
    Y = np.array(y)

    print(X.shape)
    print(X.dtype)
    print("X is finite:", np.isfinite(X))
    # print(y.shape)
    print(Y.dtype)
    print("Y is finite:", np.isfinite(Y))
    # run the gridsearch
    estimator.fit(X=X, y=Y)

    if hasattr(estimator, "best_score_"):
        return estimator.best_params_
        # print("best_score R² = {:.3f}".format(estimator.best_score_))
        # print("best_params = ", estimator.best_params_)

    # if hasattr(estimator, "best_score_"):
    #     print("best_score R² = {:.3f}".format(estimator.best_score_))
    #     print("best_params = ", estimator.best_params_)
    #
    # print("\nCV results::")
    # if hasattr(estimator, "cv_results_"):
    #     for key in [
    #         "mean_test_score",
    #         "mean_train_score",
    #         "param_method",
    #         "param_variogram_model",
    #     ]:
    #         print(" - {} : {}".format(key, estimator.cv_results_[key]))


if __name__ == "__main__":
    # dummy data
    data_x = np.random.randint(0, 400, size=(100, 2)).astype(float)
    data_y = 5 * np.random.rand(100)

    best_params = kriging_param_cv(data_x, data_y)
    print(best_params)

    # # run the gridsearch
    # estimator.fit(X=X, y=y)
    #
    # if hasattr(estimator, "best_score_"):
    #     print("best_score R² = {:.3f}".format(estimator.best_score_))
    #     print("best_params = ", estimator.best_params_)
    #
    # print("\nCV results::")
    # if hasattr(estimator, "cv_results_"):
    #     for key in [
    #         "mean_test_score",
    #         "mean_train_score",
    #         "param_method",
    #         "param_variogram_model",
    #     ]:
    #         print(" - {} : {}".format(key, estimator.cv_results_[key]))