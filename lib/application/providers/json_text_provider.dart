import 'dart:convert';

import 'package:flutter/services.dart';
import 'package:flutter_riverpod/flutter_riverpod.dart';

final jsonTextProvider =
    FutureProvider<dynamic>((ref) async {
  var jsonText = await rootBundle.loadString('assets/texte/app_content.json');
  return json.decode(jsonText);
});
