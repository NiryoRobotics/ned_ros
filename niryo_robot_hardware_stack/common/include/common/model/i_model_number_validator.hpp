/*
i_model_number_validator.hpp
Copyright (C) 2026 Niryo
All rights reserved.
This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.
This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.
You should have received a copy of the GNU General Public License
along with this program.  If not, see <http:// www.gnu.org/licenses/>.
*/

#ifndef I_MODEL_NUMBER_VALIDATOR_HPP
#define I_MODEL_NUMBER_VALIDATOR_HPP

#include <cstdint>
#include <string>
#include <vector>
#include <set>
#include <algorithm>

namespace common
{
namespace model
{
/**
 * @brief Interface for model number validation strategies
 * 
 * This abstraction allows different hardware families to define their own
 * validation rules, supporting exact matches, ranges, or sets of valid model numbers.
 */
class IModelNumberValidator
{
public:
    virtual ~IModelNumberValidator() = default;

    /**
     * @brief Check if a model number is valid according to this validator's rules
     * @param model_number The model number to validate
     * @return true if valid, false otherwise
     */
    virtual bool isValid(uint16_t model_number) const = 0;

    /**
     * @brief Get a human-readable description of valid model numbers
     * @return Description string for logging/debugging
     */
    virtual std::string describe() const = 0;
};

/**
 * @brief Validates exact match to a single model number
 * 
 * Use this for hardware with a single, fixed model number.
 */
class ExactModelNumberValidator : public IModelNumberValidator
{
private:
    uint16_t _expected;

public:
    explicit ExactModelNumberValidator(uint16_t expected) : _expected(expected) {}

    bool isValid(uint16_t model_number) const override
    {
        return model_number == _expected;
    }

    std::string describe() const override
    {
        return "exact model " + std::to_string(_expected);
    }
};

/**
 * @brief Validates model numbers within a continuous range (inclusive)
 * 
 * Use this for hardware families with sequential model number variants.
 */
class RangeModelNumberValidator : public IModelNumberValidator
{
private:
    uint16_t _min;
    uint16_t _max;

public:
    RangeModelNumberValidator(uint16_t min, uint16_t max) : _min(min), _max(max) {}

    bool isValid(uint16_t model_number) const override
    {
        return model_number >= _min && model_number <= _max;
    }

    std::string describe() const override
    {
        return "range [" + std::to_string(_min) + "-" + std::to_string(_max) + "]";
    }
};

/**
 * @brief Validates model numbers from a discrete set
 * 
 * Use this for hardware families with non-sequential model numbers
 * or when only specific variants are supported.
 */
class SetModelNumberValidator : public IModelNumberValidator
{
private:
    std::set<uint16_t> _valid_numbers;

public:
    explicit SetModelNumberValidator(std::initializer_list<uint16_t> valid_numbers)
        : _valid_numbers(valid_numbers) {}

    explicit SetModelNumberValidator(const std::vector<uint16_t>& valid_numbers)
        : _valid_numbers(valid_numbers.begin(), valid_numbers.end()) {}

    bool isValid(uint16_t model_number) const override
    {
        return _valid_numbers.find(model_number) != _valid_numbers.end();
    }

    std::string describe() const override
    {
        std::string desc = "set {";
        bool first = true;
        for (uint16_t num : _valid_numbers)
        {
            if (!first) desc += ", ";
            desc += std::to_string(num);
            first = false;
        }
        desc += "}";
        return desc;
    }
};

/**
 * @brief Always accepts any model number (no validation)
 * 
 * Use this for testing or simulation where model number checking
 * should be bypassed entirely.
 */
class AnyModelNumberValidator : public IModelNumberValidator
{
public:
    bool isValid(uint16_t model_number) const override
    {
        (void)model_number;  // Unused
        return true;
    }

    std::string describe() const override
    {
        return "any model number";
    }
};

} // namespace model
} // namespace common

#endif // I_MODEL_NUMBER_VALIDATOR_HPP
