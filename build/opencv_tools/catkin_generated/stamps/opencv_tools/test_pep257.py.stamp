# Copyright 2015 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import pydocstyle

@pytest.mark.linter
@pytest.mark.pep257
def test_pep257():
    style_guide = pydocstyle.StyleGuide()
    result = style_guide.check_files(['.'])
    assert result.total_errors == 0, f'Found {result.total_errors} docstring errors / warnings'