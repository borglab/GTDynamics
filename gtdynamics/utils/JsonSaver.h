/*!
 * \brief visualization functions for factor graph
 * \author Yetong. Zhang, zhangyetong@holomatic.com
 * \date June-14-2019
 * \attention Copyright Holomatic Technology (Beijing) Co.Ltd
 * \attention Please refer to COPYRIGHT.txt for complete terms of copyright
 * notice.
 */

#ifndef GTDYNAMICS_UTILS_JSONSAVER_H_
#define GTDYNAMICS_UTILS_JSONSAVER_H_

#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/slam/PriorFactor.h>

#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <typeinfo>
#include <vector>
#include <map>
#include <utility>

#include "gtdynamics/factors/PoseFactor.h"
#include "gtdynamics/factors/TorqueFactor.h"
#include "gtdynamics/factors/TwistAccelFactor.h"
#include "gtdynamics/factors/TwistFactor.h"
#include "gtdynamics/factors/WrenchEquivalenceFactor.h"
#include "gtdynamics/factors/WrenchFactors.h"
#include "gtdynamics/factors/WrenchPlanarFactor.h"
#include "gtdynamics/utils/utils.h"

#include <boost/optional.hpp>

#define kQuote_ "\""

using namespace gtdynamics;
namespace gtsam {
/**
 * @brief
 * store optimization results history, export factor graph in json format. The
 * format is as follows: [[{}, {}, ...], [{}, {}, ...]]. The first list contains
 * variables, and the second list contains factors. Each variable/list is
 * represented as a dictionary, with each of its attribute as an element of the
 * dictionary. The attributes for variables include: [name, value,
 * value_history, location] The attributes for factors include [name, type,
 * variables, measurement, noise, error, whitened_error]
 */
class JsonSaver {
 public:
  typedef std::pair<std::string, std::string> AttributeType;
  typedef boost::shared_ptr<gtsam::Value> ValuePtr;
  typedef std::map<Key, gtsam::Vector3> LocationType;
  typedef std::map<std::string, gtsam::Vector3> StrLocationType;

  /**
   * @brief constructor
   */
  JsonSaver() {}

  /**
   * @brief add double quotes to string
   * @param[in] str           input string
   * @return                  a string with double quotes
   */
  static inline std::string Quoted(const std::string& str) {
    return kQuote_ + str + kQuote_;
  }

  /**
   * @brief convert vector to a list in json format
   * @param[in] vec           gtsam vector
   * @return                  a string representing vector in json format
   */
  static inline std::string GetVector(const gtsam::Vector& vec) {
    std::stringstream ss;
    ss << "[";
    for (auto i = 0; i < vec.size(); i++) {
      ss << vec[i];
      if (i + 1 < vec.size()) {
        ss << ", ";
      }
    }
    ss << "]";
    return ss.str();
  }

  /**
   * @brief convert vector to a list in json format
   * @param[in] vec           gtsam vector
   * @return                  a string representing vector in json format
   */
  static inline std::string GetName(const gtsam::Key& key) {
    std::stringstream ss;
    auto symb = LabeledSymbol(key);
    char ch = symb.chr();
    int index = symb.label();
    int t = symb.index();
    if (ch == 'F') {
      ss << ch << int(index / 16) << index % 16 << "_" << t;
    } else {
      ss << ch << index << "_" << t;
    }
    return ss.str();
  }

  /**
   * @brief combine key value pairs into a dict in json format
   * @param[in] items         key value paris
   * @param[in] num_indents   number of indents for each item, -1 for no
   * indent and no newline
   * @return                  a string representing dict in json format
   */
  static inline std::string JsonDict(const std::vector<AttributeType>& items,
                                     const int num_indents = 0) {
    std::string indents;
    std::string s;
    if (num_indents < 0) {
      indents = "";
      s = "{";
    } else {
      indents = "\n" + std::string(num_indents, ' ');
      s = std::string(num_indents, ' ') + "{";
    }
    for (auto& item : items) {
      s += indents + item.first + ":" + item.second + ",";
    }
    s.pop_back();  // remove the last comma
    s += indents + "}";
    return s;
  }

  /**
   * @brief combine items into a list in json format
   * @param[in] items         vector of items
   * @param[in] num_indents   number of indents for each item, -1 for no
   * indent and no newline
   * @return                  a string representing list in json format
   */
  static inline std::string JsonList(const std::vector<std::string>& items,
                                     const int num_indents = 0) {
    std::string indents;
    std::string s;
    if (num_indents < 0) {
      indents = "";
      s = "[";
    } else {
      indents = "\n" + std::string(num_indents, ' ');
      s = std::string(num_indents, ' ') + "[";
    }
    for (auto& item : items) {
      s += indents + item + ",";
    }
    s.pop_back();  // remove the last comma
    s += indents + "]";
    return s;
  }

  /**
   * @brief get the gtsam variable value as a string
   * @param[in] value         gtsam variable value
   * @return                  a string displaying the value
   */
  static inline std::string GetValue(const gtsam::Value& value) {
    std::stringstream ss;

    // pose variable
    if (const gtsam::GenericValue<gtsam::Pose3>* p =
            dynamic_cast<const gtsam::GenericValue<gtsam::Pose3>*>(&value)) {
      ss << "Translation: " << p->value().translation();
      ss << " Rotation rpy: [" << p->value().rotation().rpy()[0] << ", "
         << p->value().rotation().rpy()[1] << ", "
         << p->value().rotation().rpy()[2] << "]";
    } else if (const gtsam::GenericValue<gtsam::Vector6>* p =
                 dynamic_cast<const gtsam::GenericValue<gtsam::Vector6>*>(
                     &value)) {
      // Wrench, Twist, Twistaccel variable
      ss << p->value().transpose();
    } else if (const gtsam::GenericValue<gtsam::Vector3>* p =
                 dynamic_cast<const gtsam::GenericValue<gtsam::Vector3>*>(
                     &value)) {
      // velocity, acceleration variables
      ss << p->value().transpose();
    } else if (const gtsam::GenericValue<gtsam::Vector>* p =
                 dynamic_cast<const gtsam::GenericValue<gtsam::Vector>*>(
                     &value)) {
      // general vector
      ss << p->value().transpose();
    } else if (const gtsam::GenericValue<double>* p =
                 dynamic_cast<const gtsam::GenericValue<double>*>(&value)) {
      // Torque, q, v, a
      ss << p->value();
    }
    return ss.str();
  }

  /**
   * @brief get measurements of the factor as a string
   * @param[in] factor        gtsam factor pointer
   * @return                  a string displaying the measurement
   */
  static inline std::string GetMeasurement(
      const gtsam::NonlinearFactor::shared_ptr& factor) {
    std::stringstream ss;
    if (const TorqueFactor* f =
            dynamic_cast<const TorqueFactor*>(&(*factor))) {
      ss << GetVector(f->getScrewAxis().transpose());
    } else if (const PriorFactor<Vector3>* f =
                   dynamic_cast<const PriorFactor<Vector3>*>(&(*factor))) {
      ss << GetVector(f->prior().transpose());
    } else if (const PriorFactor<Vector6>* f =
                   dynamic_cast<const PriorFactor<Vector6>*>(&(*factor))) {
      ss << GetVector(f->prior().transpose());
    } else if (const PriorFactor<double>* f =
                   dynamic_cast<const PriorFactor<double>*>(&(*factor))) {
      ss << f->prior();
    }
    return ss.str();
  }

  /**
   * @brief get measurements of the factor as a string
   * @param[in] factor        gtsam factor pointer
   * @return                  a string displaying the measurement
   */
  static inline std::string GetType(
      const gtsam::NonlinearFactor::shared_ptr& factor) {
    if (dynamic_cast<const WrenchFactor1*>(&(*factor))) {
      return "Wrench";
    } else if (dynamic_cast<const WrenchFactor2*>(&(*factor))) {
      return "Wrench";
    } else if (dynamic_cast<const WrenchFactor3*>(&(*factor))) {
      return "Wrench";
    } else if (dynamic_cast<const WrenchFactor4*>(&(*factor))) {
      return "Wrench";
    } else if (dynamic_cast<const PoseFactor*>(&(*factor))) {
      return "Pose";
    } else if (dynamic_cast<const TwistFactor*>(&(*factor))) {
      return "Twist";
    } else if (dynamic_cast<const TwistAccelFactor<ScrewJointBase>*>(&(*factor))) {
      return "TwistAccel";  // TODO(G+S): figure out how to make this work for other joint types
    } else if (dynamic_cast<const TorqueFactor*>(&(*factor))) {
      return "Torque";
    } else if (dynamic_cast<const WrenchPlanarFactor*>(&(*factor))) {
      return "WrenchPlanar";
    } else if (dynamic_cast<const WrenchEquivalenceFactor*>(&(*factor))) {
      return "WrenchEq";
    } else if (dynamic_cast<const PriorFactor<double>*>(&(*factor))) {
      return "Prior";
    } else if (dynamic_cast<const PriorFactor<Vector>*>(&(*factor))) {
      return "Prior";
    } else if (dynamic_cast<const PriorFactor<Pose3>*>(&(*factor))) {
      return "PriorPose";
    } else {
      return typeid(factor).name();
    }
  }

  /**
   * @brief get the gtsam noise model of the factor as a string
   * @param[in] factor        gtsam factor pointer
   * @return                  a string displaying the noise
   */
  static inline std::string GetNoiseModel(
      const gtsam::NonlinearFactor::shared_ptr& factor) {
    std::stringstream ss;
    if (const gtsam::NoiseModelFactor* noise_factor =
            dynamic_cast<const gtsam::NoiseModelFactor*>(&(*factor))) {
      const gtsam::noiseModel::Base::shared_ptr noise_model =
          noise_factor->noiseModel();
      // unit
      if (const gtsam::noiseModel::Unit* true_noise_model =
              dynamic_cast<const gtsam::noiseModel::Unit*>(&(*noise_model))) {
        ss << "unit " << true_noise_model->isUnit();
      } else if (const gtsam::noiseModel::Isotropic* true_noise_model =
                   dynamic_cast<const gtsam::noiseModel::Isotropic*>(
                       &(*noise_model))) {
        // isotropic
        ss << boost::format("isotropic dim=%1% sigma=%2%") %
                  true_noise_model->dim() % true_noise_model->sigma();
      } else if (const gtsam::noiseModel::Constrained* true_noise_model =
                   dynamic_cast<const gtsam::noiseModel::Constrained*>(
                       &(*noise_model))) {
        // constrained
        ss << "constrained mus:" << GetVector(true_noise_model->mu());
      } else if (const gtsam::noiseModel::Diagonal* true_noise_model =
                   dynamic_cast<const gtsam::noiseModel::Diagonal*>(
                       &(*noise_model))) {
        // diagonal
        ss << "diagonal sigmas: " << GetVector(true_noise_model->sigmas());
      } else if (const gtsam::noiseModel::Gaussian* true_noise_model =
                   dynamic_cast<const gtsam::noiseModel::Gaussian*>(
                       &(*noise_model))) {
        // gaussian
        ss << "gaussian sigmas: " << GetVector(true_noise_model->sigmas());
      } else {
        ss << typeid(noise_model).name();
      }
    }
    return ss.str();
  }

  /**
   * @brief get the gtsam error of the factor as a string
   * @param[in] factor        gtsam factor pointer
   * @param[in] values        gtsam values of variables
   * @return                  a string displaying the error
   */
  static inline std::string GetError(
      const gtsam::NonlinearFactor::shared_ptr& factor, const Values& values) {
    std::stringstream ss;
    ss << factor->error(values);
    return ss.str();
  }

  /**
   * @brief get the gtsam whitened error of the factor as a string
   * @param[in] factor        gtsam factor pointer
   * @param[in] values        gtsam values of variables
   * @return                  a string displaying the whitened error
   */
  static inline std::string GetWhitenedError(
      const gtsam::NonlinearFactor::shared_ptr& factor, const Values& values) {
    std::stringstream ss;
    if (const gtsam::NoiseModelFactor* noise_factor =
            dynamic_cast<const gtsam::NoiseModelFactor*>(&(*factor))) {
      ss << GetVector(noise_factor->whitenedError(values));
    }
    return ss.str();
  }

  /**
   * @brief get the location of the variable
   * @param[in] locations     manually specified locations
   * @param[in] key           variable key
   * @param[in] value         value of varible
   * @return                  a string displaying the location
   */
  static inline std::string GetLocation(const LocationType& locations,
                                        const gtsam::Key& key,
                                        const gtsam::Value& value) {
    std::stringstream ss;
    if (locations.size() > 0) {
      if (locations.find(key) != locations.end()) {
        ss << GetVector(locations.at(key));
      }
    } else {
      // pose variable
      if (const gtsam::GenericValue<gtsam::Pose3>* p =
              dynamic_cast<const gtsam::GenericValue<gtsam::Pose3>*>(&value)) {
        ss << GetVector(p->value().translation());
      } else if (const gtsam::GenericValue<gtsam::Point3>* p =
                   dynamic_cast<const gtsam::GenericValue<gtsam::Point3>*>(
                       &value)) {
        // landmark variable
        ss << GetVector(p->value());
      }
    }
    return ss.str();
  }

  /**
   * @brief get the variable in json format as a string
   * @param[in] key           corresponding key of variable
   * @param[in] values        values
   * @param[in] locations     locations
   * @return                  a string displaying the variable in json
   */
  static inline std::string GetVariable(const gtsam::Key& key,
                                        const gtsam::Values& values,
                                        const LocationType& locations) {
    std::vector<AttributeType> attributes;

    // name;
    std::string name = GetName(key);
    attributes.emplace_back(Quoted("name"), Quoted(name));

    if (values.exists(key)) {
      // value
      boost::optional<gtsam::Vector3> location;
      attributes.emplace_back(Quoted("value"),
                              Quoted(GetValue(values.at(key))));

      // location
      const auto loc_str = GetLocation(locations, key, values.at(key));
      if (loc_str != "") {
        attributes.emplace_back(Quoted("location"), loc_str);
      }
    }
    return JsonDict(attributes);
  }

  /**
   * @brief get the factor in json format as a string
   * @param[in] idx           index of factor
   * @param[in] graph         factor graph
   * @param[in] values        values
   * @return                  a string displaying the factor in json
   */
  static inline std::string GetFactor(const size_t idx,
                                      const NonlinearFactorGraph& graph,
                                      const gtsam::Values& values) {
    const gtsam::NonlinearFactor::shared_ptr& factor = graph.at(idx);

    std::vector<AttributeType> attributes;

    // name
    attributes.emplace_back(Quoted("name"),
                            Quoted("Factor" + std::to_string(idx)));

    // type
    attributes.emplace_back(Quoted("type"), Quoted(GetType(factor)));

    // variables
    const gtsam::KeyVector& keys = factor->keys();
    std::vector<std::string> variable_names;
    for (gtsam::Key key : keys) {
      std::stringstream ss;
      ss << Quoted(GetName(key));
      variable_names.push_back(ss.str());
    }
    attributes.emplace_back(Quoted("variables"), JsonList(variable_names, -1));

    // measurement
    attributes.emplace_back(Quoted("measurement"),
                            Quoted(GetMeasurement(factor)));

    // noise model
    attributes.emplace_back(Quoted("noise"), Quoted(GetNoiseModel(factor)));

    // whitened noise model
    attributes.emplace_back(Quoted("whitened error"),
                            Quoted(GetWhitenedError(factor, values)));

    // error
    attributes.emplace_back(Quoted("error"), GetError(factor, values));

    return JsonDict(attributes);
  }

  /**
   * @brief output the json format factor graph to ostream
   * @param[in] graph         gtsam factor graph
   * @param[in] stm           output stream
   * @param[in] values        gtsam values of variables
   * @param[in] locations     manually specify the location of variables
  // TODO: add option to include GT values
   */
  static inline void SaveFactorGraph(
      const NonlinearFactorGraph& graph, std::ostream& stm,
      const gtsam::Values& values = gtsam::Values(),
      const LocationType& locations = LocationType()) {
    std::vector<std::string> variable_strings;
    std::vector<std::string> factor_strings;

    // add variables
    for (gtsam::Key key : graph.keys()) {
      variable_strings.push_back(GetVariable(key, values, locations));
    }

    // add factors
    for (size_t i = 0; i < graph.size(); ++i) {
      factor_strings.push_back(GetFactor(i, graph, values));
    }

    std::string s_variables = JsonList(variable_strings);
    std::string s_factors = JsonList(factor_strings);
    std::vector<std::string> all_strings{s_variables, s_factors};
    std::string s_all = JsonList(all_strings);
    stm << s_all;
  }

  /**
   * @brief get the gtsam variable value as a string in list format
   * @param[in] value         gtsam variable value
   * @return                  a string displaying the value
   */
  static inline std::string GetValueList(const gtsam::Value& value) {
    std::stringstream ss;

    // pose variable
    if (const gtsam::GenericValue<gtsam::Pose3>* p =
            dynamic_cast<const gtsam::GenericValue<gtsam::Pose3>*>(&value)) {
      ss << "[" << p->value().translation().x() << ", "
         << p->value().translation().y() << ", " << p->value().translation().z()
         << ", " << p->value().rotation().rpy()[0] << ", "
         << p->value().rotation().rpy()[1] << ", "
         << p->value().rotation().rpy()[2] << "]";
    }
    return ss.str();
  }

  /**
   * @brief get names of the types of the value, e.g. ["x", "y", "z"]
   * @param[in] value         gtsam variable value
   * @return                  a string displaying the value types
   */
  static inline std::string GetValueTypes(const gtsam::Value& value) {
    // pose variable
    std::vector<std::string> value_types;
    if (typeid(value) == typeid(gtsam::GenericValue<gtsam::Pose3>)) {
      value_types =
          std::vector<std::string>{"x", "y", "z", "row", "pitch", "yaw"};
    } else if (typeid(value) == typeid(gtsam::GenericValue<gtsam::Point3>)) {
      // landmark variable
      value_types = std::vector<std::string>{"x", "y", "z"};
    }
    for (auto& value_type : value_types) {
      value_type = Quoted(value_type);
    }
    return JsonList(value_types, -1);
  }

  static inline void SaveClusteredGraph(std::ostream &stm,
                                        const std::map<std::string, gtsam::NonlinearFactorGraph> &clustered_graphs,
                                        const std::map<std::string, gtsam::Values> &clustered_values,
                                        const gtsam::Values &values,
                                        const StrLocationType &locations = StrLocationType())
  {
    // create map from key to value_cluster name for faster searching
    std::map<gtsam::Key, std::string> key_to_cluster;
    for (auto value_cluster : clustered_values)
    {
      std::string cluster_name = value_cluster.first;
      // std::cout << "value cluster: " << cluster_name << "\n";
      for (const auto &key : value_cluster.second.keys())
      {
        key_to_cluster[key] = cluster_name;
      }
    }

    std::vector<std::string> values_strings;
    std::vector<std::string> graphs_strings;

    // add clustered values
    for (const auto &it : clustered_values)
    {
      std::string cluster_name = it.first;
      const gtsam::Values &values = it.second;

      std::vector<AttributeType> attributes;
      // name
      attributes.emplace_back(Quoted("name"),
                              Quoted(cluster_name));

      // location
      if (locations.find(cluster_name) != locations.end())
      {
        const auto loc_str = GetVector(locations.at(cluster_name));
        attributes.emplace_back(Quoted("location"), loc_str);
      }

      // values
      std::vector<std::string> varaible_names;
      for (const gtsam::Key &key : values.keys())
      {
        varaible_names.emplace_back(GetName(key));
      }
      attributes.emplace_back(JsonSaver::Quoted("value"), Quoted(JsonList(varaible_names, -1)));
      values_strings.emplace_back(JsonDict(attributes));
    }

    // add clustered graphs
    for (const auto &it : clustered_graphs)
    {
      std::string cluster_name = it.first;
      const gtsam::NonlinearFactorGraph &graph = it.second;

      // std::cout << "graph cluster: " << cluster_name << "\tsize:" << graph.size() << "\n";

      std::vector<AttributeType> attributes;
      // name
      attributes.emplace_back(Quoted("name"),
                              Quoted(cluster_name));

      // varaible clusters
      std::set<std::string> values_cluster_names;
      for (const auto &factor : graph)
      {
        for (const auto &key : factor->keys())
        {
          values_cluster_names.insert(Quoted(key_to_cluster[key]));
        }
      }
      std::vector<std::string> vec_cluster_names(values_cluster_names.begin(), values_cluster_names.end());
      attributes.emplace_back(Quoted("variables"), JsonList(vec_cluster_names, -1));

      // calculate errors
      double error = 0;
      for (const auto &factor : graph)
      {
        error += factor->error(values);
      }
      attributes.emplace_back(Quoted("error"), std::to_string(error));

      // location
      if (locations.find(cluster_name) != locations.end())
      {
        const auto loc_str = GetVector(locations.at(cluster_name));
        attributes.emplace_back(Quoted("location"), loc_str);
      }

      graphs_strings.emplace_back(JsonDict(attributes));
    }

    std::string s_variables = JsonList(values_strings);
    std::string s_factors = JsonList(graphs_strings);
    std::vector<std::string> all_strings{s_variables, s_factors};
    std::string s_all = JsonList(all_strings);
    stm << s_all;
  }
};

class StorageManager {
 private:
  typedef JsonSaver::AttributeType AttributeType;
  typedef boost::shared_ptr<gtsam::Value> ValuePtr;
  typedef std::map<gtsam::Key, std::vector<ValuePtr>> StorageMap;
  typedef std::pair<gtsam::Key, std::vector<ValuePtr>> StorageEntry;
  StorageMap storage_;

 public:
  /**
   * @brief constructor
   */
  StorageManager() {}

  /**
   * @brief get the history of values e.g. [[x_t0, y_t0, z_t0], [x_t1, y_t1,
   * z_t1], [x_t2, y_t2, z_t2]]
   * @param[in] key   key for the variable
   * return           the history values of the variable
   */
  std::string GetValueHistory(const gtsam::Key& key) {
    std::vector<std::string> value_sequence;
    for (const auto value_ptr : storage_.at(key)) {
      value_sequence.push_back(JsonSaver::GetValueList(*value_ptr));
    }
    return JsonSaver::JsonList(value_sequence, -1);
  }

  /**
   * @brief get the variable in json format as a string
   * @param[in] key           corresponding key of variable
   * @param[in] locations     locations
   * @return                  a string displaying the variable in json
   */
  std::string GetVariableSequence(const gtsam::Key& key,
                                  const JsonSaver::LocationType& locations) {
    std::vector<AttributeType> attributes;

    // name
    std::string name = JsonSaver::GetName(key);
    attributes.emplace_back(JsonSaver::Quoted("name"), JsonSaver::Quoted(name));

    if (storage_.find(key) != storage_.end()) {
      // value
      attributes.emplace_back(
          JsonSaver::Quoted("value"),
          JsonSaver::Quoted(JsonSaver::GetValue(*(storage_.at(key).back()))));

      // value history
      attributes.emplace_back(JsonSaver::Quoted("value_history"),
                              GetValueHistory(key));

      // value type
      attributes.emplace_back(
          JsonSaver::Quoted("value_types"),
          JsonSaver::GetValueTypes(*(storage_.at(key).back())));

      // location
      const auto loc_str =
          JsonSaver::GetLocation(locations, key, *(storage_.at(key).back()));
      if (loc_str != "") {
        attributes.emplace_back(JsonSaver::Quoted("location"), loc_str);
      }
    }
    return JsonSaver::JsonDict(attributes);
  }

  /**
   * @brief output the json format factor graph (sequence) to ostream
   * @param[in] graph         gtsam factor graph
   * @param[in] stm           output stream
   * @param[in] values        gtsam values of variables
   * @param[in] locations     manually specify the location of variables
   */
  void SaveFactorGraphSequence(
      const NonlinearFactorGraph& graph, std::ostream& stm,
      const gtsam::Values& values = gtsam::Values(),
      const JsonSaver::LocationType& locations = JsonSaver::LocationType()) {
    std::vector<std::string> variable_strings;
    std::vector<std::string> factor_strings;

    // add variables
    for (gtsam::Key key : graph.keys()) {
      variable_strings.push_back(GetVariableSequence(key, locations));
    }

    // add factors
    for (size_t i = 0; i < graph.size(); ++i) {
      factor_strings.push_back(JsonSaver::GetFactor(i, graph, values));
    }

    std::string s_variables = JsonSaver::JsonList(variable_strings);
    std::string s_factors = JsonSaver::JsonList(factor_strings);
    std::vector<std::string> all_strings{s_variables, s_factors};
    std::string s_all = JsonSaver::JsonList(all_strings);
    stm << s_all;
  }

  /**
   * @brief                   add values to storage
   * @param[in] values        gtsam values of variables in the current step
   */
  void AddValues(const gtsam::Values& values) {
    // remove absent variables
    for (auto key_value : storage_) {
      if (!values.exists(key_value.first)) {
        storage_.erase(key_value.first);
      }
    }

    // update existing variables
    for (auto key : values.keys()) {
      auto it = storage_.find(key);
      if (it == storage_.end()) {
        storage_.emplace(key, std::vector<ValuePtr>{values.at(key).clone()});
      } else {
        it->second.push_back(values.at(key).clone());
      }
    }
  }
};

// const std::string JsonSaver::kQuote_ = "\"";

}  // namespace gtsam

#endif   // GTDYNAMICS_UTILS_JSONSAVER_H_
