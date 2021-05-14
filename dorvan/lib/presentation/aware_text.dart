import 'package:dorvan/application/providers/language_state_provider.dart';
import 'package:dorvan/application/providers/translated_text_provider.dart';
import 'package:flutter/material.dart';
import 'package:flutter_riverpod/flutter_riverpod.dart';

class AwareText extends StatelessWidget {
  final String? text;
  final Widget Function(String text)? builder;

  AwareText(this.text, {@required this.builder});

  Widget build(BuildContext context) {
    return Consumer(
      builder: (context, watch, child) {
        final translatedText = watch(translatedTextProvider)[text] ?? '-';

        return builder!(translatedText);
      },
    );
  }
}
